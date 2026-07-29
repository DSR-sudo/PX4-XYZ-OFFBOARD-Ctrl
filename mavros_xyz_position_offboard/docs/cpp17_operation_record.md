# C++17 Operation Record: UAV JSON V2 Refactor

## Components

| Path | Responsibility |
| --- | --- |
| `application/` | Single `ApplicationNode`, fixed 20 Hz loop, LCP Debug subscription, Init-Z latching, component assembly. |
| `communication/` | Strict V2 value types, non-blocking fixed-remote UDP, source allowlist, ordered event ACK queue, `xyzstatus` encoder. |
| `navigation/` | ROS-free V2 mission state machine and bounded XY/Z quintic planner. |
| `gripper/` | Non-blocking Pi Linux PWM sysfs adapter with testable chip-path root. |
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
6. Write audit status (`px4.mavros_native_xyz.v2`).

`/lcp/debug` is callback-driven rather than timer-driven: each new `lslidar_msgs/LcpDebug` sample
is copied into one `xyzstatus` datagram immediately. It is independent of event retries.

## Parameters

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `udp.event_retry_period_s` | `0.5` | Retry period for the oldest unacknowledged `ok_*`. |
| `--setpoint-warmup` / `setpoint_warmup_s` | `2.0` | Init-hold setpoint publication time before `ok_wait`. |
| `mission.takeoff_height_m` | `1.5` | Relative MAVROS local-Z climb from Init. |
| `mission.right_shift_m` | `0.375` | Initial-heading right shift; startup validation restricts it to 0.35--0.40 m. |
| `mission.forward_distance_m` | `5.0` | Bounded straight-line distance after GCS `go_ahead_ok`. |
| `mission.match_hold_seconds` | `0.5` | Wait at the GCS-confirmed match point before release. |
| `z.prefer_range` | `false` | Select Range-relative Z after field calibration. |
| `z.source_timeout_s` | `0.5` | Freshness limit for pose/Range Z sources. |
| `z.range_cross_check_max_delta_m` | `0.30` | Maximum local-vs-Range relative-Z disagreement. |
| `gripper_pwm.enabled` | `false` | Enables physical sysfs output only after calibration. |
| `gripper_pwm.release_delay_ms` | `500` | Delay before changing to release duty. |

The existing `udp.bind_*`, `remote_*`, and `whitelist_*` parameters are unchanged. The YAML
also defines PWM `chip_path`, `channel`, `period_ns`, idle/release duty, hold time, and pinmux
check fields.

## PWM Procedure

1. Keep `gripper_pwm.enabled:false` for all normal software builds and SITL.
2. With propellers removed, identify the Linux PWM chip/channel and verify pinmux manually.
3. Measure safe idle and release pulse widths for the actual gripper; set a period larger than
   both duties.
4. Set `pinmux_path` to the readable pinctrl inspection file and `pinmux_expected` to the exact
   expected token. Set a non-production temporary chip root first when testing.
5. Enable PWM and verify exported channel, `period`, `duty_cycle`, `enable`, release delay,
   hold, and idle restoration before connecting a payload.

When enabled, preparation performs pinmux text validation, chip/channel existence or export,
sysfs permission/open checks, period write, idle duty write, and enable. Any failure reports
`failed`, does not send `ok_throw`, and resumes bounded pursuit for a fresh
`match_car_ok` retry. No automatic test uses `/sys/class/pwm`.

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
- Duplicated `run_plan1`, `match_car_ok`, and `b_ok` do not repeat ARM, PWM release, return, or
  landing actions.
- `go_ahead_ok` is accepted only after the bounded 0.35--0.40 m right shift has completed.
- `match_car_ok` is accepted only during bounded forward pursuit; the GCS owns black-line, car,
  and `<0.1 m` distance verification.
- LCP failure during climb does not interrupt climb. Later LCP failure holds the measured
  position until recovery, then resumes the interrupted final target; mode/flight-health loss
  and max-flight timeout retain their existing safe paths.
- Normal completion means descend to Init, PX4 Disarm, and `MANUAL`; it never shuts down the Pi
  or removes FCU power.
