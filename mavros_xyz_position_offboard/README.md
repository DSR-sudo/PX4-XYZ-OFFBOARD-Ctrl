# ROS 2 MAVROS UAV Joint-Mission Node

This package is the UAV-side ROS 2 Jazzy node for the joint GCS/UAV task. It keeps the
existing explicit MAVROS authorization gates, OFFBOARD setpoint streaming, bounded quintic
trajectories, LCP health hold, maximum-flight-time protection, normal Disarm, and return to
`MANUAL`. It replaces the old JSON V1 navigation batches completely.

The wire contract is [GCS_UAV_JSON通信协议.md](/home/pi/GCS_UAV_JSON通信协议.md). There is no
V1 fallback.

## Runtime Model

```text
GCS UDP -> GroundStationLink -> Navigation -> Offboard -> MAVROS/PX4
                 ^                   |
LCP /lcp/debug --+                   +-> PwmGripper
Initialization -- health/Z ----------+
```

- `GroundStationLink` accepts one UTF-8 JSON object per UDP datagram, validates the source
  IP/port allowlist and the complete Plan1 object shape, and sends only to the fixed remote.
- `Navigation` is a pure C++ state machine. It keeps an immutable ARM-time `origin`, a task
  `mission_goal`, the published `commanded_setpoint`, and a separate temporary `hold_setpoint`.
  It flies to the fixed local ENU B point, then converts each GCS visual relative measurement into
  a vehicle-center ENU target without turning toward the vehicle.
- `Offboard` sends internal ENU position/yaw setpoints unchanged to MAVROS `PositionTarget`.
  MAVROS 2.14 `SetpointRawPlugin::local_cb()` converts the ROS ENU input to PX4 `LOCAL_NED`;
  the application must not convert it a second time.
- `LcpVisionBridge` still publishes converted LCP external-vision pose. The application also
  subscribes to `lslidar_msgs/LcpDebug` on `/lcp/debug` and sends one `xyzstatus` per sample.
- `PwmGripper` is a non-blocking SG90 adapter built on `lgpio`. When enabled it dynamically finds
  the Pi 5 RP1 controller, drives BCM GPIO18 at 50 Hz, and holds the verified 4% closed position;
  disabled mode remains a timed no-op for SITL.

Mission order:

```text
waiting_preflight -> ground-hold warmup -> ok_wait -> waiting_run_plan1/OFFBOARD hold
-> run_plan1 -> ARM -> latch origin and climb 1.5 m -> height_stabilizing (3 s)
-> transit_to_b -> ok_b -> waiting_target -> target_lock_following (10 s dynamic follow)
-> waiting_target/throwing
-> PWM release -> ok_throw -> returning -> Init XY + yaw 0 -> ok_return
-> downing -> ok_downing -> descend Init Z -> Disarm -> MANUAL -> ok_down
```

An LCP loss during climb does not interrupt the climb. A later LCP loss freezes the measured
position until LCP recovers, then replans to the interrupted final target. Loss of mode, flight
health, or the maximum flight time still goes to the existing safe landing path. “Power off” in
the mission documents means PX4 Disarm plus `MANUAL`, never powering down the Pi or FCU.

## UDP V3

Every packet has exactly this root shape:

```json
{"header":"<name>","data":{}}
```

GCS sends `run_plan1` and `ack` with an empty `data` object. It sends continuous visual
measurements after acknowledging `ok_b`:

```json
{"header":"car_status","data":{"distance_m":0.8,"bearing_rad":0.15}}
```

`distance_m` is in `[0, udp.max_tracking_distance_m]` (default `5.0`) and `bearing_rad` is in
`[-pi, pi]`; it is the target line-of-sight angle relative to the UAV body: zero means straight
ahead, `+pi/2` is the UAV's left, and `+/-pi` is directly behind.
The first accepted `car_status` starts `target_lock_following` for
`mission.target_lock_follow_seconds` (default `10.0 s`). During this one-time phase every accepted
observation is converted with the current measured yaw and becomes the dynamic vehicle-center XY
target, but the raw distance gate cannot release the gripper. The mission altitude and ARM-time yaw
are preserved, and each new observation replans the XY target in place with the two-dimensional resultant limits
`mission.car_tracking_max_speed_m_s` (default `10.0 m/s`) and
`mission.car_tracking_max_accel_m_s2` (default `5.0 m/s2`). If either parameter is omitted, its
effective `safety.target_xy_*` value is inherited. Return uses the independent
`mission.return_max_speed_m_s` (default `10.0 m/s`) and
`mission.return_max_accel_m_s2` (default `5.0 m/s2`) limits, inheriting the effective
`safety.target_xy_*` values when omitted; the example YAML sets them to `1.0/0.5`. Normal
`downing` uses the independent Z limits `mission.downing_max_speed_m_s` and
`mission.downing_max_accel_m_s2` (both default `0.3`), while failure `landing` keeps the global
Z limits. B-point and landing XY trajectories keep using the global `safety.target_xy_*` limits.
During `waiting_target` and `target_lock_following`, consecutive finite `local_z` samples are
compared for a short jump. A downward jump at least `mission.tracking_z_jump_threshold_m`
(default `0.10 m`) within `mission.tracking_z_jump_window_s` (default `0.10 s`) lowers the
temporary planner Z target by `mission.tracking_z_step_m` (default `0.11 m`); an upward jump
applies the opposite step. The accumulated offset is preserved through visual/LCP hold recovery,
while the original `origin.z_m` and `mission_goal.z_m` remain unchanged. New jumps are ignored
while waiting for the gripper in `throwing`, and the offset is cleared on return, landing,
descent, or mission reset. The offset and latest jump direction are included in `control_json`.
After the lock-follow timer
completes, the latest fresh raw `distance_m < mission.throw_distance_m` (default `0.20 m`)
immediately enters `throwing`; equality does not release. Subsequent car_status messages use the
normal live distance gate. `throw_bearing_rad` and `throw_bearing_tolerance_rad` remain accepted compatibility
parameters but do not participate in target planning or release.

UAV discrete events are `ok_wait`, `ok_b`, `ok_throw`, `ok_return`, `ok_downing`, and `ok_down`.
They are ordered in an ACK queue. The UAV sends and retransmits only the earliest unsatisfied event
every `udp.event_retry_period_s` (default `0.5`); `{"header":"ack","data":{}}` removes only that
head. `xyzstatus` is continuous and is neither queued nor acknowledged. Legacy `ok_height`,
`go_ahead_ok`, `match_car_ok`, and `b_ok` are rejected as `legacy_header_rejected`.

At ARM, the UAV latches Init and initial yaw for height/orientation control. After the 3-second
stability gate it holds the climb altitude and flies directly to the fixed local ENU B point:

```text
xB = mission.b_right_m   # default 0.375 m; historical parameter name, ENU X
yB = mission.b_forward_m # default 2.375 m; historical parameter name, ENU Y
```

The XY move is one two-dimensional trajectory. `ok_b` is emitted once when that planned trajectory
is complete; B does not add a measured XYZ or velocity settling gate. Every
accepted `car_status` uses the UAV receipt time and measured yaw to form the raw ENU vehicle-center
target. A large in-radius jump is accepted and replans immediately; the active Navigation path does
not use Kalman samples, innovation rejection, cardinal shaping, or predicted intercept timing.
Stale or out-of-radius observations still enter the existing safe LCP hold.

The height compensation is an application-level setpoint adjustment. It does not change the
`car_status` UDP message or the continuous `xyzstatus` wire format, and `sensor_msgs/Range` remains
an input to the existing health checks. It cannot undo an already fused PX4 EKF2 downward-range
measurement; isolating that fusion would require a separate PX4/MAVROS input-chain change.

On a successful gripper cycle, `ok_throw` is queued and the UAV immediately returns without a GCS
command. It emits `ok_return` only after Init XY and yaw zero are measured, then queues
`ok_downing`, descends, Disarms, returns to `MANUAL`, and emits `ok_down`. See
[`docs/plan1_intercept_udp_protocol.md`](../docs/plan1_intercept_udp_protocol.md) for the complete
wire protocol and idempotency rules.

## LCP and Z

`xyzstatus` copies the raw LCP header and every geometry field from `/lcp/debug`. During preflight
warmup and GCS wait, the node streams fresh measured ground-hold setpoints without creating a
flight origin. After `ok_wait`, the GCS wait also requests OFFBOARD while remaining disarmed;
`run_plan1` enables ARM only after OFFBOARD is confirmed. Only ARM confirmation in OFFBOARD locks the MAVROS local pose as `Init` and starts
the local-Z/range baseline. Default Z is the fresh local-pose difference from that baseline. A
fresh valid down range is baselined at the same point and checks relative Z consistency
(`z.range_cross_check_max_delta_m`, default `0.30 m`); set
`z.prefer_range:true` only after field validation.

When no selected source is fresh, valid, and consistent, LCP is still sent with exactly:

```json
"position_z_m":null,"z_source":"none","z_source_stamp":null,"z_quality":null,"z_valid":false
```

## Build and Test

`lslidar_msgs` is required. Source the LSLIDAR overlay before configure/build/test:

```bash
cd /home/pi/px4-test-tools
source /opt/ros/jazzy/setup.bash
source /home/pi/LSLIDARN10P/install/setup.bash
colcon build --packages-select mavros_xyz_position_offboard --parallel-workers 1
source install/setup.bash
colcon test --packages-select mavros_xyz_position_offboard --event-handlers console_direct+
colcon test-result --all --verbose
```

The tests cover V3 validation, non-whitelisted inputs, ordered ACK retry state, full LCP
`xyzstatus` encoding including null Z, state-machine mission order, mocked `lgpio` PWM calls, and UDP
loopback. No automated test drives a real PWM pin.

## Configuration and PWM Calibration

Load [config/udp_ground_station.yaml](config/udp_ground_station.yaml) with ROS parameters. It
contains the UDP allowlist/fixed remote, the `mission.takeoff_height_m: 1.5` and
`mission.height_stable_seconds: 3.0` gate, B-point, vehicle-center tracking, retained Kalman,
tracking-height jump compensation, Z, and PWM defaults. The vehicle-center speed and acceleration limits are explicitly set to
`10.0 m/s` and `5.0 m/s2`; without those mission entries, the node inherits the effective safety
XY limits. The
3-second timer accumulates only while measured XYZ remains within `--target-tolerance`; leaving
the tolerance restarts the timer.

## Unified Bringup

Use the package launch file to start MAVROS, the N10P LSLIDAR driver, and this application under
one ROS 2 launch supervisor:

```bash
cd /home/pi/px4-test-tools
source /opt/ros/jazzy/setup.bash
source /home/pi/LSLIDARN10P/install/setup.bash
source install/setup.bash
ros2 launch mavros_xyz_position_offboard flight_stack.launch.py
```

Its MAVROS defaults are the flight-controller serial URL
`/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:2000000` and GCS URL
`udp://127.0.0.1:14551@`. It includes `lslidar_driver/lsn10p_launch.py`; once MAVROS telemetry,
the N10P LCP service, and the disarmed/on-ground state are healthy, the application automatically
calls `/lcp/start_initialization`. This permits safe no-battery LCP commissioning. Battery,
position, range, optical-flow, estimator, and LCP-ready checks remain mandatory before the task
can warm the Init setpoint, emit `ok_wait`, or accept a flight command. A missing service is
retried only while the ground conditions remain valid.

The unified launch does not enable position setpoints, OFFBOARD requests, or arming. Those remain
behind the existing explicit command-line confirmations, so starting the stack is suitable for
the no-battery communication and sensor checks. Do not start this launch while another MAVROS or
LSLIDAR process already owns the same serial device or ROS interfaces. When a verified MAVROS
instance is already connected, retain it and start only the LSLIDAR/application portion with
`start_mavros:=false`. For an isolated no-battery sensor check, also use `udp_enabled:=false`.

If preflight becomes invalid after `run_plan1` and before ARM is confirmed, the navigation state
clears the task and reuses its existing `manual_request_pending` safety path to request Disarm plus
`MANUAL`. It emits the machine-readable rejection `preflight_lost_before_arm`. Once ARM is
confirmed, preflight is no longer used: loss of OFFBOARD or flight health keeps the vehicle armed
and requests `AUTO.LAND`. Audit records include both `preflight_errors` and `flight_errors` for
diagnosis.

The deployed YAML enables the calibrated SG90 setup: BCM GPIO18 (physical pin 12), 50 Hz, 4%
closed, 7% open, and a 500 ms open hold. Before flight, verify this on a no-prop bench with the
signal wire connected to GPIO18 and ensure the launch user can access the RP1 gpiochip through the
`dialout` group. If RP1 discovery, GPIO claim, or initial 4% PWM setup fails, node startup fails.
After the one-time target lock-follow phase, the raw 0.20 m vehicle-center gate opens the gripper.
Successful completion of
its configured open/close cycle emits `ok_throw` and immediately begins autonomous return.

Install the C++ GPIO dependency on deployment images with `sudo apt install liblgpio-dev`.

Run in this order: no-prop bench and standalone PWM calibration, PX4 SITL, then controlled flight.
Capture UDP datagrams and `/lcp/debug`/MAVROS timestamps during acceptance.

## Flight Authorization

The prior CLI gates remain mandatory. The node does not create a setpoint publisher, mode client,
or arming client until the corresponding explicit confirmations are supplied. A typical monitored
launch remains:

```bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node \
  --confirmed-fcu-url udp://127.0.0.1:14540 \
  --range-topic /mavros/px4flow/ground_distance --range-source-label downward \
  --optical-flow-topic /mavros/px4flow/raw/optical_flow_rad --optical-flow-source-label flow \
  --ros-args --params-file \
  /home/pi/px4-test-tools/install/mavros_xyz_position_offboard/share/mavros_xyz_position_offboard/config/udp_ground_station.yaml
```

Add the established safety acknowledgements only for the applicable bench/SITL/controlled-flight
stage. Passing unit tests is not flight authorization.
