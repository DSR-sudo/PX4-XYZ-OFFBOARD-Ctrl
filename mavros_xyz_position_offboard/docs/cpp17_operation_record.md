# C++17 Operation Record: UAV JSON V2 Refactor

## Components

| Path | Responsibility |
| --- | --- |
| `application/` | Single `ApplicationNode`, fixed 20 Hz loop, LCP Debug subscription, Init-Z latching, component assembly. |
| `communication/` | Strict V2 value types, non-blocking fixed-remote UDP, source allowlist, ordered event ACK queue, `xyzstatus` encoder. |
| `navigation/` | ROS-free V2 mission state machine and bounded XY/Z quintic planner. |
| `gripper/` | Non-blocking SG90 adapter using `lgpio` and dynamic Pi 5 RP1 gpiochip discovery. |
| `initialization/` | Existing MAVROS telemetry, preflight/flight health, LCP start, RangeGuard, and ROS source timestamps. |
| `bridge/` | Existing LCP NWU-to-ENU external-vision publisher. |
| `config/udp_ground_station.yaml` | UDP, mission motion, Z source, and PWM parameters. |

No module creates an internal ROS topic. The ROS executor remains single-threaded.

## Control Loop

Each timer cycle retains this order:

1. Poll LCP-start/MAVROS service futures and readable UDP datagrams.
2. Capture one immutable health snapshot and request LCP initialization when eligible.
3. Advance PWM state and call `Navigation::update` with health, protocol events, and gripper result.
4. Apply the idempotent setpoint/mode/arm intent to MAVROS.
5. Start a requested PWM release, queue each `ok_*`, and retry the earliest queued event.
6. Write audit status (`px4.mavros_native_xyz.v3`).

`/lcp/debug` is callback-driven rather than timer-driven: each new `lslidar_msgs/LcpDebug` sample
is copied into one `xyzstatus` datagram immediately. It is independent of event retries.

## Parameters

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `udp.event_retry_period_s` | `0.5` | Retry period for the oldest unacknowledged `ok_*`. |
| `udp.max_tracking_distance_m` | `5.0` | Maximum accepted `car_status.distance_m`. |
| `--setpoint-warmup` / `setpoint_warmup_s` | `2.0` | Init-hold setpoint publication time before `ok_wait`. |
| `mission.takeoff_height_m` | `1.5` | Relative MAVROS local-Z climb from Init. |
| `mission.right_shift_m` | `0.375` | Initial-heading right shift; startup validation restricts it to 0.35--0.40 m. |
| `mission.forward_distance_m` | `5.0` | Bounded straight-line distance after GCS `go_ahead_ok`. |
| `mission.forward_max_speed_m_s` | `0.50` | XY speed limit for the `go_ahead_ok` forward search; startup validation rejects values above `safety.max_flight_horizontal_speed_m_s`. |
| `mission.tracking_arrival_seconds` | `1.0` | Requested arrival time for each continuously replanned visual XY target. |
| `mission.tracking_tolerance_m` | `0.2` | Fresh visual distance required before `match_car_ok` confirms release. |
| `mission.match_hold_seconds` | `0.5` | Measured-position hold between a confirmed match and gripper release. |
| `mission.car_status_timeout_s` | `2.0` | Time without a valid visual update before measured-position hold. |
| `mission.max_tracking_radius_m` | `5.0` | Maximum visual target radius from the locked Init XY origin. |
| `mission.accompanying_z_jump_threshold_m` | `0.10` | Minimum abrupt EKF local-Z change to compensate after release. |
| `mission.accompanying_z_step_m` | `0.11` | Temporary Z setpoint adjustment per detected local-Z jump. |
| `mission.accompanying_z_jump_window_s` | `0.10` | Maximum sample interval for a local-Z jump to qualify. |
| `z.prefer_range` | `false` | Select Range-relative Z after field calibration. |
| `z.source_timeout_s` | `0.5` | Freshness limit for pose/Range Z sources. |
| `z.range_cross_check_max_delta_m` | `0.30` | Maximum local-vs-Range relative-Z disagreement. |
| `gripper_pwm.enabled` | `true` | Enables the calibrated physical SG90 output; override to `false` for SITL. |
| `gripper_pwm.bcm_gpio` | `18` | SG90 signal GPIO (physical pin 12). |
| `gripper_pwm.pwm_frequency_hz` | `50.0` | SG90 PWM frequency. |
| `gripper_pwm.closed_duty_cycle` | `4.0` | Verified closed position duty cycle, percent. |
| `gripper_pwm.open_duty_cycle` | `7.0` | Verified open position duty cycle, percent. |
| `gripper_pwm.open_hold_ms` | `500` | Time to hold the open signal before restoring closed. |

`car_status` is the only nonempty inbound message: its exact data object is
`{"distance_m": <0..5>, "bearing_rad": <-pi..pi>}`. The existing `udp.bind_*`, `remote_*`, and
`whitelist_*` parameters are unchanged. The planner records whether each requested visual arrival
time was met; it extends the trajectory instead of exceeding configured speed or acceleration.

## PWM Procedure

1. Install `liblgpio-dev`; run SITL with `gripper_pwm.enabled:false`.
2. With propellers removed, start the enabled node and verify that RP1 discovery claims BCM GPIO18
   and immediately outputs 50 Hz / 4%.
3. Confirm the SG90 opens at 7% and restores 4% after 500 ms. Keep the launch user in `dialout`
   so it can access the RP1 gpiochip.

When enabled, node startup finds the RP1 gpiochip from its sysfs label, opens it with `lgpio`,
claims BCM GPIO18, and starts the 4% closed PWM. Startup failures stop the node before a mission
can begin. After the confirmed match hold, navigation commands the adapter to release. It emits
`ok_throw` only after the configured open/close cycle succeeds, then resumes visual accompaniment.

## Build and Acceptance

```bash
cd /home/pi/px4-test-tools
source /opt/ros/jazzy/setup.bash
source /home/pi/LSLIDARN10P/install/setup.bash
colcon build --packages-select mavros_xyz_position_offboard --parallel-workers 1
source install/setup.bash
colcon test --packages-select mavros_xyz_position_offboard --event-handlers console_direct+
colcon test-result --all --verbose
```

The source overlay is required because this package depends on `lslidar_msgs`. Execute field
acceptance in three stages: no-prop/PWM bench, PX4 SITL, then controlled flight. Capture both
UDP datagrams and ROS timestamps from `/lcp/debug`, local pose, and Range to verify packet field
values and the event order.

## Safety Invariants

- Existing CLI acknowledgements remain the only route to setpoint publication, mode request, and
  normal ARM/Disarm requests.
- `run_plan1` does not start setpoint warmup; Init is latched and held for the configured warmup
  before `ok_wait`, so `run_plan1` directly requests OFFBOARD/ARM and climb.
- Duplicated `run_plan1`, `match_car_ok`, and `b_ok` do not repeat ARM, gripper release, return, or landing actions.
- `go_ahead_ok` is accepted only after the bounded 0.35--0.40 m right shift has completed.
- The first valid `car_status` changes bounded forward pursuit to visual ENU tracking. Every later
  status uses actual local XY and current yaw, keeps the task height/yaw, and is rejected if its
  target exceeds the Init-radius bound.
- `match_car_ok` is accepted only in `tracking_to_match` with a fresh distance no greater than
  `mission.tracking_tolerance_m`; it freezes at the measured position for
  `mission.match_hold_seconds`, then starts PWM release. Once release succeeds, `ok_throw` is sent
  and fresh visual updates continue in `accompanying_car` until `b_ok`. A visual timeout freezes at
  the measured position and a fresh status restores the interrupted tracking stage.
- During post-release accompaniment only, a sudden EKF local-Z step caused by the downward laser
  intercepting the target shifts the temporary Z setpoint by 0.11 m in the same direction. Return,
  landing, and gripper failure clear this compensation.
- LCP failure during climb does not interrupt climb. Later LCP failure holds the measured
  position until recovery, then resumes the interrupted final target; mode/flight-health loss
  and max-flight timeout retain their existing safe paths.
- Normal completion means descend to Init, PX4 Disarm, and `MANUAL`; it never shuts down the Pi
  or removes FCU power.
