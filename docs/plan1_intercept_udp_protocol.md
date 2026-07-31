# Plan1 Vehicle-Center Tracking UDP Protocol

## Scope

This document defines the Plan1 UDP protocol for B-point direct transit, raw vehicle-center
tracking, payload release, autonomous return, and landing. The retained Kalman implementation is
tested independently and is not used by the active Navigation path. Each UDP
datagram is one UTF-8 JSON object with exactly these envelope fields:

```json
{"header":"<message name>","data":{}}
```

The UAV accepts packets only from its configured IP and port allowlist. Unknown
envelope members, duplicate keys, wrong data shapes, non-finite numbers, wrong
direction headers, and legacy headers are rejected. The four legacy headers
`ok_height`, `go_ahead_ok`, `match_car_ok`, and `b_ok` are rejected with the
machine-readable reason `legacy_header_rejected`.

## Messages

| Direction | Header | Data | Allowed UAV phase | Rule |
| --- | --- | --- | --- | --- |
| GCS -> UAV | `run_plan1` | `{}` | `waiting_run_plan1` | Starts the normal OFFBOARD/ARM/climb sequence. Repeated packets in later startup phases are idempotent. |
| GCS -> UAV | `car_status` | `{"distance_m": number, "bearing_rad": number}` | `waiting_target` or recoverable `lcp_hold` | `distance_m` is metres and `bearing_rad` is the target line-of-sight angle relative to measured UAV yaw: zero forward, `+pi/2` left, and `+/-pi` rear. GCS sends no timestamp; UAV receipt time is the measurement time. |
| GCS -> UAV | `ack` | `{}` | Any phase | Confirms exactly the earliest queued UAV `ok_*` event. An ACK with no queued event is harmless. |
| UAV -> GCS | `ok_wait` | `{}` | after preflight and setpoint warmup | GCS may send `run_plan1` after receiving it. |
| UAV -> GCS | `ok_b` | `{}` | `waiting_target` | Sent once only after the two-dimensional B trajectory is complete, measured XYZ is within tolerance, and measured horizontal/vertical speeds are below `mission.b_arrival_speed_m_s`. GCS starts graphic recognition and continuous `car_status` after receiving it. |
| UAV -> GCS | `ok_throw` | `{}` | `returning` | Sent once immediately after a successful servo release; no GCS command is required for return. |
| UAV -> GCS | `ok_return` | `{}` | `downing` | Sent after measured Init XY and yaw `0` are restored. |
| UAV -> GCS | `ok_downing` | `{}` | `downing` | Sent as the normal descent begins. |
| UAV -> GCS | `ok_down` | `{}` | `manual` | Sent after landing, ordinary Disarm confirmation, and `MANUAL` confirmation. |
| UAV -> GCS | `xyzstatus` | LCP status object | Any | Continuous telemetry. It is never ACKed. |

`car_status.distance_m` must be in `[0, udp.max_tracking_distance_m]` and
`bearing_rad` must be in `[-pi, pi]`. UAV transforms it at receipt to ENU:

```text
car_x = uav_x + distance_m * cos(uav_measured_yaw + bearing_rad)
car_y = uav_y + distance_m * sin(uav_measured_yaw + bearing_rad)
```

Every accepted observation writes `car_x/car_y` to `mission_goal` and calls
`planner.set_xy_target_with_limits(car_x, car_y, mission.car_tracking_max_speed_m_s,
mission.car_tracking_max_accel_m_s2)`. These are independent two-dimensional resultant limits;
when the mission parameters are omitted, they inherit the effective `safety.target_xy_*` values.
The task altitude and ARM-time yaw are preserved, and a new observation immediately replaces the
previous XY target. B-point and landing trajectories continue to use the global safety limits.
Return instead uses `mission.return_max_speed_m_s` and `mission.return_max_accel_m_s2`;
when omitted, each inherits the effective `safety.target_xy_*` value. It still targets the ARM-time
Init XY at the current altitude and commands world yaw zero. `bearing_rad` is used only for this ENU conversion. When the latest valid and fresh raw `distance_m < mission.throw_distance_m` (default
`0.20 m`), the UAV enters `throwing` and requests the gripper in the same control cycle. The strict
comparison means `distance_m == 0.20 m` does not release. The JSON fields and UDP envelope are
unchanged.

## ACK And Idempotency

Each `ok_*` event enters one FIFO queue in creation order. The UAV sends the
queue head immediately and retransmits that same packet at
`udp.event_retry_period_s` until an `ack` removes it. Only after the head is
acknowledged can the next event be transmitted. Repeated event datagrams are
therefore expected and must be ACKed again; GCS treats each header as idempotent.
`xyzstatus` is not in this queue and must not be acknowledged.

`run_plan1` is idempotent during the pending OFFBOARD/ARM startup path. A
`car_status` is a fresh observation rather than a command and may be sent at a fixed rate. An
observation outside the configured Init tracking radius or a stale stream enters the existing
`lcp_hold`. In-radius jumps are accepted as raw targets; Navigation does not apply Kalman innovation
rejection or predicted-intercept timing.

## Flight Sequence

```text
UAV: preflight + Init hold -> ok_wait
GCS:                                ack, run_plan1
UAV: OFFBOARD + ARM + climb -> height_stabilizing (continuous 3 s)
UAV: transit_to_b
UAV: at B within tolerance -> ok_b -> waiting_target
GCS:                         ack, begin recognition and continuous car_status
UAV: raw vehicle-center tracking -> throwing
UAV: servo success -> ok_throw -> returning (without a GCS command)
GCS:                   ack
UAV: Init XY + yaw 0 -> ok_return -> downing -> ok_downing
GCS:                      ack                       ack
UAV: touchdown -> Disarm -> MANUAL -> ok_down
GCS:                                      ack
```

The B point uses fixed local ENU coordinates while holding the climb altitude and
initial yaw. The historical parameter names are retained for YAML compatibility:

```text
xB = mission.b_right_m   # default 0.375 m
yB = mission.b_forward_m # default 2.375 m
```

The B arrival gate requires the trajectory planner to be complete, measured XYZ
within `target_tolerance_m`, and measured horizontal and vertical speeds each no
greater than `mission.b_arrival_speed_m_s` (default `0.05 m/s`). The trajectory
planner performs one two-dimensional XY plan; it does not split the move into
forward and right legs. After B, every accepted observation directly targets the
converted vehicle center while preserving task altitude and initial yaw. Release
uses only the latest fresh raw distance and the strict `<0.20 m` threshold; the
retained bearing parameters are not used.
