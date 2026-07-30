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
- `Navigation` is a pure C++ state machine. It keeps an immutable ARM-time `origin`, a task
  `mission_goal`, the published `commanded_setpoint`, and a separate temporary `hold_setpoint`.
  It plans the bounded right shift and straight pursuit from the ARM-time heading; GCS vision
  remains responsible for alignment and car-distance decisions.
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
waiting_preflight -> ground-hold warmup -> ok_wait -> waiting_run_plan1
-> run_plan1 -> OFFBOARD/ARM -> latch origin and climb 1.5 m -> hold stable for 3 s -> ok_height
-> right-shift 0.375 m -> wait for go_ahead_ok -> bounded forward pursuit
-> GCS distance match (<0.1 m) -> hold 0.5 s -> PWM release -> ok_throw -> b_ok -> ok_return
-> return Init XY + world yaw 0 -> ok_downing -> descend Init Z -> Disarm -> MANUAL -> ok_down
```

An LCP loss during climb does not interrupt the climb. A later LCP loss freezes the measured
position until LCP recovers, then replans to the interrupted final target. Loss of mode, flight
health, or the maximum flight time still goes to the existing safe landing path. “Power off” in
the mission documents means PX4 Disarm plus `MANUAL`, never powering down the Pi or FCU.

## UDP V2

Every packet has exactly this root shape:

```json
{"header":"<name>","data":{}}
```

GCS sends `run_plan1`, `go_ahead_ok`, `match_car_ok`, `b_ok`, and `ack` with an empty `data`
object. `car_status` is no longer an accepted UAV command.

UAV discrete events are `ok_wait`, `ok_height`, `ok_throw`, `ok_return`, `ok_downing`, and
`ok_down`. They are ordered in an ACK queue. The UAV sends the earliest unsatisfied event right
away and retries it every `udp.event_retry_period_s` (default `0.5`). A GCS
`{"header":"ack","data":{}}` acknowledges only that earliest event. `xyzstatus` is continuous
and is neither queued nor acknowledged.

GCS may apply its own map or business condition before sending `b_ok`. The UAV knows only its
relative local coordinates, accepts `b_ok` in `awaiting_b_ok`, and then returns to its latched
Init XY origin. `position_z_m`, `z_valid`, and the selected Z source are unrelated to `b_ok`.

After `ok_height`, the UAV translates `mission.right_shift_m` (validated to 0.35--0.40 m) to its
initial-heading right side and waits. The GCS must compare the initial and translated `xyzstatus`
coordinates and independently verify that the black line is centered in its video before sending
`go_ahead_ok`. The UAV then flies its bounded `mission.forward_distance_m` path. The GCS owns car
recognition and the `<0.1 m` distance decision; it sends `match_car_ok` only after that check.
The UAV holds the confirmed position for `mission.match_hold_seconds` (default `0.5`) before it
immediately drives the verified 7% open position. It holds that signal for
`gripper_pwm.open_hold_ms` (default `500`) before restoring the 4% closed position and sending
`ok_throw`. Duplicate commands are phase-idempotent and cannot re-arm, release twice, or restart
return.

## LCP and Z

`xyzstatus` copies the raw LCP header and every geometry field from `/lcp/debug`. During preflight
warmup and GCS wait, the node streams fresh measured ground-hold setpoints without creating a
flight origin. Only ARM confirmation in OFFBOARD locks the MAVROS local pose as `Init` and starts
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

The tests cover V2 validation, non-whitelisted inputs, ordered ACK retry state, full LCP
`xyzstatus` encoding including null Z, state-machine mission order, mocked `lgpio` PWM calls, and UDP
loopback. No automated test drives a real PWM pin.

## Configuration and PWM Calibration

Load [config/udp_ground_station.yaml](config/udp_ground_station.yaml) with ROS parameters. It
contains the UDP allowlist/fixed remote, the `mission.takeoff_height_m: 1.5` and
`mission.height_stable_seconds: 3.0` gate, right-shift/forward/match defaults, Z, and PWM
defaults. The 3-second timer only
accumulates while measured XYZ remains within `--target-tolerance`; leaving the tolerance returns
the state machine to climb and restarts the timer.

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
`dialout` group. If RP1 discovery, GPIO claim, or initial 4% PWM setup fails, node startup fails;
if a later open/close PWM command fails, the release is withheld and a new `match_car_ok` may retry.

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
