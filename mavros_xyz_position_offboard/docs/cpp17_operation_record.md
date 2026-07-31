# C++17 Operation Record: Plan1 Intercept

## Components

| Path | Responsibility |
| --- | --- |
| `application/` | Single `ApplicationNode`, fixed 20 Hz loop, LCP Debug subscription, Init-Z latching, and component assembly. |
| `communication/` | Strict Plan1 value types, fixed-remote UDP, source allowlist, ordered ACK queue, and `xyzstatus` encoding. |
| `navigation/navigation.hpp` | Compatibility aggregate header and ROS-free mission state-machine coordinator. |
| `navigation/mission_config.*` | Mission defaults and configuration validation. |
| `navigation/trajectory_planner.*` | Bounded quintic XYZ+yaw trajectories and continuous replanning. |
| `navigation/target_tracker.*` | 2D constant-velocity Kalman filtering and intercept-time calculation. |
| `navigation/navigation_types.*` | Controller inputs, auditable control state/decisions, and `control_json`. |
| `gripper/` | Non-blocking SG90 adapter using `lgpio` and dynamic Pi 5 RP1 gpiochip discovery. |
| `initialization/` | MAVROS telemetry, preflight/flight health, LCP start, RangeGuard, and ROS source timestamps. |
| `bridge/` | LCP NWU-to-ENU external-vision publisher. |

The ROS executor remains single-threaded. No module creates an internal ROS topic.

## Control Loop

Each timer cycle polls service futures and UDP, captures one immutable health snapshot, advances
the gripper and `Navigation`, applies setpoint/mode/arm intent, sends queued business events, then
writes audit status. Every `/lcp/debug` sample is separately sent as one unqueued `xyzstatus`.

## Plan1 Parameters

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `udp.event_retry_period_s` | `0.5` | Retry period for the earliest unacknowledged `ok_*`. |
| `udp.max_tracking_distance_m` | `5.0` | Maximum accepted `car_status.distance_m`. |
| `mission.takeoff_height_m` | `1.5` | Relative MAVROS local-Z climb from Init. |
| `mission.height_stable_seconds` | `3.0` | Required continuous XYZ stability before B-point transit. |
| `mission.b_right_m` | `0.375` | B-point right component in the initial body frame. |
| `mission.b_forward_m` | `2.375` | B-point forward component in the initial body frame. |
| `mission.throw_distance_m` | `0.20` | Predicted separation that permits release. |
| `mission.filter_measurement_noise_m` | `0.05` | Target position measurement standard deviation. |
| `mission.filter_acceleration_noise_m_s2` | `0.50` | Constant-velocity filter acceleration process-noise standard deviation. |
| `mission.filter_min_samples` | `3` | Valid observations required before prediction. |
| `mission.prediction_horizon_s` | `2.0` | Maximum look-ahead for predicted intercept. |
| `mission.cardinal_tolerance_deg` | `5.0` | Tolerance around nearest relative 0/90/180/270 degree direction. |
| `mission.final_intercept_seconds` | `0.5` | Window where cardinal translation shaping is removed. |
| `mission.car_status_timeout_s` | `2.0` | Freshness bound before safe visual hold. |
| `mission.max_tracking_radius_m` | `5.0` | Maximum target radius from Init XY. |

`car_status` is the only nonempty inbound message. Its `bearing_rad` is relative to measured UAV
yaw, zero is forward, and positive is counter-clockwise. `Navigation` converts it at receipt to
ENU, filters `[target_x, target_y, target_vx, target_vy]`, and computes the first predicted time
within `throw_distance_m` using measured UAV horizontal velocity.

## Safety And Completion

The normal flight phases are `height_stabilizing`, `transit_to_b`, `waiting_target`,
`cardinal_alignment`, `final_intercept`, `throwing`, `returning`, `downing`, and `manual`.
Additional preflight, OFFBOARD/ARM, LCP-hold, landing, Disarm, and MANUAL-request phases retain
the established safety boundaries.

At ARM, Init and `psi0` are latched. B is planned once at the climb altitude and initial yaw:

```text
xB = x0 + 2.375*cos(psi0) + 0.375*sin(psi0)
yB = y0 + 2.375*sin(psi0) - 0.375*cos(psi0)
```

Only trajectory completion plus measured B tolerance emits `ok_b`. A stale visual sample, target
outside the Init-radius bound, innovation outlier, or unavailable intercept clears release timing
and freezes the safe hold point. Successful PWM release queues `ok_throw` and begins return
without a GCS command. `ok_return` is emitted only at Init XY and yaw zero; `ok_downing` starts
descent; `ok_down` requires touchdown, confirmed ordinary Disarm, and confirmed `MANUAL`.

`ok_wait`, `ok_b`, `ok_throw`, `ok_return`, `ok_downing`, and `ok_down` use the ordered FIFO ACK
queue. The legacy headers `ok_height`, `go_ahead_ok`, `match_car_ok`, and `b_ok` are rejected with
`legacy_header_rejected`.

## Build And Acceptance

```bash
cd /home/pi/px4-test-tools
source /opt/ros/jazzy/setup.bash
source /home/pi/LSLIDARN10P/install/setup.bash
colcon build --packages-select mavros_xyz_position_offboard --parallel-workers 1
source install/setup.bash
colcon test --packages-select mavros_xyz_position_offboard --event-handlers console_direct+
colcon test-result --all --verbose
```

The source overlay provides `lslidar_msgs`. Field acceptance remains staged: no-prop PWM bench,
PX4 SITL, then controlled flight. The full wire contract is in
[`../../docs/plan1_intercept_udp_protocol.md`](../../docs/plan1_intercept_udp_protocol.md).
