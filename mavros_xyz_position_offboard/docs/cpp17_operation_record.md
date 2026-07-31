# C++17 Operation Record: Plan1 Vehicle-Center Tracking

## Components

| Path | Responsibility |
| --- | --- |
| `application/` | Single `ApplicationNode`, fixed 20 Hz loop, LCP Debug subscription, Init-Z latching, and component assembly. |
| `communication/` | Strict Plan1 value types, fixed-remote UDP, source allowlist, ordered ACK queue, and `xyzstatus` encoding. |
| `navigation/navigation.hpp` | Compatibility aggregate header and ROS-free mission state-machine coordinator. |
| `navigation/navigation.cpp` | Navigation object lifecycle, mission-goal normalization, and planner orchestration. |
| `navigation/navigation_state_machine.cpp` | Phase transitions, protocol event dispatch, health gates, and cycle decisions. |
| `navigation/navigation_mission.cpp` | B-point planning, raw visual target conversion, and return planning. |
| `navigation/navigation_safety.cpp` | Position/yaw stability checks, LCP hold/resume, and landing safety decisions. |
| `navigation/mission_config.*` | Mission defaults and configuration validation. |
| `navigation/trajectory_planner.*` | Bounded quintic XYZ+yaw trajectories and continuous replanning. |
| `navigation/target_tracker.*` | Retained standalone 2D constant-velocity Kalman filtering and intercept-time tests; not used by active Navigation tracking. |
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
| `mission.b_right_m` | `0.375` | Fixed local ENU X coordinate of B; historical parameter name retained for compatibility. |
| `mission.b_forward_m` | `2.375` | Fixed local ENU Y coordinate of B; historical parameter name retained for compatibility. |
| `mission.b_arrival_speed_m_s` | `0.05` | Maximum measured horizontal resultant speed and vertical speed for B arrival. |
| `mission.car_tracking_max_speed_m_s` | `10.0` | Maximum resultant XY speed for `car_status` vehicle-center tracking; inherits `safety.target_xy_max_speed_m_s` when omitted. |
| `mission.car_tracking_max_accel_m_s2` | `5.0` | Maximum resultant XY acceleration for `car_status` vehicle-center tracking; inherits `safety.target_xy_max_accel_m_s2` when omitted. |
| `mission.return_max_speed_m_s` | `10.0` | Maximum resultant XY speed for return to the ARM-time origin; inherits `safety.target_xy_max_speed_m_s` when omitted. |
| `mission.return_max_accel_m_s2` | `5.0` | Maximum resultant XY acceleration for return to the ARM-time origin; inherits `safety.target_xy_max_accel_m_s2` when omitted. |
| `mission.throw_distance_m` | `0.20` | Strict raw distance threshold for immediate release. |
| `mission.throw_bearing_rad` | `1.57079632679` | Retained compatibility parameter; unused by vehicle-center tracking. |
| `mission.throw_bearing_tolerance_rad` | `0.08` | Retained compatibility parameter; unused by vehicle-center tracking. |
| `mission.filter_measurement_noise_m` | `0.05` | Retained standalone Kalman measurement-noise parameter. |
| `mission.filter_acceleration_noise_m_s2` | `0.50` | Retained standalone Kalman acceleration-noise parameter. |
| `mission.filter_min_samples` | `3` | Retained standalone Kalman sample-count parameter. |
| `mission.prediction_horizon_s` | `2.0` | Retained standalone Kalman query horizon. |
| `mission.cardinal_tolerance_deg` | `5.0` | Retained historical parameter; unused by active tracking. |
| `mission.final_intercept_seconds` | `0.5` | Retained historical parameter; unused by active tracking. |
| `mission.car_status_timeout_s` | `2.0` | Freshness bound before safe visual hold. |
| `mission.max_tracking_radius_m` | `5.0` | Maximum target radius from Init XY. |

`car_status` is the only nonempty inbound message. Its `bearing_rad` is the target line-of-sight
angle relative to measured UAV yaw: zero is forward, `+pi/2` is left, and `+/-pi` is rear.
`Navigation` converts it at receipt to ENU using `uav_yaw + bearing_rad`. Every valid observation
sets the raw `car_x/car_y` as both `mission_goal` XY and the planner XY target while preserving the
current task altitude and ARM-time yaw. The vehicle-center planner applies the independent
`mission.car_tracking_max_speed_m_s` and `mission.car_tracking_max_accel_m_s2` limits to the
two-dimensional resultant vector. Return planning uses the independent
`mission.return_max_speed_m_s` and `mission.return_max_accel_m_s2` limits, while B-point and
landing planning continue to use the global `safety.target_xy_*` limits. The active path does not filter, reject innovations, shape cardinal
motion, or calculate predicted intercept time. `target_samples` remains `0` and
`predicted_intercept_seconds` remains `null` in audit JSON. Strict raw `distance_m < 0.20 m` enters
`throwing` immediately; equality does not.

## Safety And Completion

The normal flight phases are `height_stabilizing`, `transit_to_b`, `waiting_target`, `throwing`,
`returning`, `downing`, and `manual`.
Additional preflight, OFFBOARD/ARM, LCP-hold, landing, Disarm, and MANUAL-request phases retain
the established safety boundaries.

At ARM, Init and `psi0` are latched. B is planned once at the climb altitude and initial yaw,
but its XY coordinates are fixed local ENU values:

```text
xB = mission.b_right_m
yB = mission.b_forward_m
```

The single `set_target()` call plans the two-dimensional XY move and Z move together; the move is
not split into forward and right legs. Only trajectory completion, measured B XYZ tolerance, and
horizontal/vertical measured speeds no greater than `mission.b_arrival_speed_m_s` emit `ok_b`. A stale visual sample or target
outside the Init-radius bound clears the active target and freezes the safe hold point; in-radius
target jumps are accepted directly. Successful PWM release queues `ok_throw` and begins return
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
