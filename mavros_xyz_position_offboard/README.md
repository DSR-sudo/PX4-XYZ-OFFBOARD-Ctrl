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
  IP/port allowlist and the complete V2 object shape, and sends only to the fixed remote.
- `Navigation` is a pure C++ state machine. It converts `car_status.distance/angle` to a
  local-ENU goal using measured vehicle position and yaw.
- `LcpVisionBridge` still publishes converted LCP external-vision pose. The application also
  subscribes to `lslidar_msgs/LcpDebug` on `/lcp/debug` and sends one `xyzstatus` per sample.
- `PwmGripper` is a non-blocking `/sys/class/pwm/pwmchip*` adapter. Its default disabled mode
  is a timed no-op for SITL; it does not touch hardware.

Mission order:

```text
waiting_preflight -> ok_wait -> waiting_run_plan1 -> warmup/OFFBOARD/ARM
-> climb 1.5 m -> ok_height -> tracking -> PWM release -> ok_throw
-> b_ok -> ok_return -> return Init XY + world yaw 0 -> ok_downing
-> descend Init Z -> Disarm -> MANUAL -> ok_down
```

An LCP loss freezes safely and then lands after the configured hold timeout. Loss of mode,
flight health, or the maximum flight time also goes to the existing safe landing path. “Power
off” in the mission documents means PX4 Disarm plus `MANUAL`, never powering down the Pi or
FCU.

## UDP V2

Every packet has exactly this root shape:

```json
{"header":"<name>","data":{}}
```

GCS sends `run_plan1`, `match_car_ok`, `b_ok`, and `ack` with an empty `data` object; it sends
`car_status` with exactly `distance` (m) and `angle` (degrees). Distance is the horizontal UAV
to vehicle separation. Angle is measured from body `+X`, counter-clockwise positive, in
`[-180, 180]`.

UAV discrete events are `ok_wait`, `ok_height`, `ok_throw`, `ok_return`, `ok_downing`, and
`ok_down`. They are ordered in an ACK queue. The UAV sends the earliest unsatisfied event right
away and retries it every `udp.event_retry_period_s` (default `0.5`). A GCS
`{"header":"ack","data":{}}` acknowledges only that earliest event. `xyzstatus` is continuous
and is neither queued nor acknowledged.

`car_status` expires after `tracking.car_status_timeout_s` (default `0.5`). The UAV freezes at
measured position rather than extrapolating. `match_car_ok` is accepted only in tracking with a
fresh status inside `tracking.standoff_m +/- tracking.match_tolerance_m`; duplicate commands are
phase-idempotent and cannot re-arm, release twice, or restart return.

## LCP and Z

`xyzstatus` copies the raw LCP header and every geometry field from `/lcp/debug`. When `ok_wait`
is emitted, the current MAVROS local Z is latched as `Init`. Default Z is the fresh local-pose
difference from that baseline. A fresh valid down range is baselined at the same point and checks
relative Z consistency (`z.range_cross_check_max_delta_m`, default `0.30 m`); set
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

The tests cover V2 validation, non-whitelisted inputs, ordered ACK retry state, full LCP
`xyzstatus` encoding including null Z, state-machine mission order, PWM sysfs fakes, and UDP
loopback. No automated test drives a real PWM pin.

## Configuration and PWM Calibration

Load [config/udp_ground_station.yaml](config/udp_ground_station.yaml) with ROS parameters. It
contains the UDP allowlist/fixed remote, tracking, Z, and PWM defaults.

Do not set `gripper_pwm.enabled:true` until a bench calibration has recorded the correct
`chip_path`, `channel`, period, idle duty, and release duty. Enabled mode also requires a readable
`gripper_pwm.pinmux_path` containing `gripper_pwm.pinmux_expected`; failure to export the PWM,
write period/duty/enable, check permissions, or verify pinmux leaves the task in tracking and
withholds `ok_throw`. A later `match_car_ok` retries the release.

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
