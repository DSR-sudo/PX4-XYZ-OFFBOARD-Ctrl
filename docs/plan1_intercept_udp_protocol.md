# Plan1 Intercept UDP Protocol

## Scope

This document defines the Plan1 UDP protocol for B-point direct transit, visual
Kalman interception, payload release, autonomous return, and landing. Each UDP
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
| GCS -> UAV | `car_status` | `{"distance_m": number, "bearing_rad": number}` | `waiting_target`, `cardinal_alignment`, `final_intercept` | `distance_m` is metres and `bearing_rad` is relative to the measured UAV yaw: zero forward, positive counter-clockwise. GCS sends no timestamp; UAV receipt time is the measurement time. |
| GCS -> UAV | `ack` | `{}` | Any phase | Confirms exactly the earliest queued UAV `ok_*` event. An ACK with no queued event is harmless. |
| UAV -> GCS | `ok_wait` | `{}` | after preflight and setpoint warmup | GCS may send `run_plan1` after receiving it. |
| UAV -> GCS | `ok_b` | `{}` | `waiting_target` | Sent once only after the diagonal trajectory is complete and measured position is within B-point tolerance. GCS starts graphic recognition and continuous `car_status` after receiving it. |
| UAV -> GCS | `ok_throw` | `{}` | `returning` | Sent once immediately after a successful servo release; no GCS command is required for return. |
| UAV -> GCS | `ok_return` | `{}` | `downing` | Sent after measured Init XY and yaw `0` are restored. |
| UAV -> GCS | `ok_downing` | `{}` | `downing` | Sent as the normal descent begins. |
| UAV -> GCS | `ok_down` | `{}` | `manual` | Sent after landing, ordinary Disarm confirmation, and `MANUAL` confirmation. |
| UAV -> GCS | `xyzstatus` | LCP status object | Any | Continuous telemetry. It is never ACKed. |

`car_status.distance_m` must be in `[0, udp.max_tracking_distance_m]` and
`bearing_rad` must be in `[-pi, pi]`. UAV transforms it at receipt to ENU:

```text
target_x = uav_x + distance_m * cos(uav_measured_yaw + bearing_rad)
target_y = uav_y + distance_m * sin(uav_measured_yaw + bearing_rad)
```

## ACK And Idempotency

Each `ok_*` event enters one FIFO queue in creation order. The UAV sends the
queue head immediately and retransmits that same packet at
`udp.event_retry_period_s` until an `ack` removes it. Only after the head is
acknowledged can the next event be transmitted. Repeated event datagrams are
therefore expected and must be ACKed again; GCS treats each header as idempotent.
`xyzstatus` is not in this queue and must not be acknowledged.

`run_plan1` is idempotent during the pending OFFBOARD/ARM startup path. A
`car_status` is a fresh observation rather than a command and may be sent at a
fixed rate. Observations outside the configured tracking radius, an innovation
outlier, a stale observation, or a prediction outside the horizon cancel the
release timer and freeze the existing safe hold position.

## Flight Sequence

```text
UAV: preflight + Init hold -> ok_wait
GCS:                                ack, run_plan1
UAV: OFFBOARD + ARM + climb -> height_stabilizing (continuous 3 s)
UAV: transit_to_b
UAV: at B within tolerance -> ok_b -> waiting_target
GCS:                         ack, begin recognition and continuous car_status
UAV: cardinal_alignment -> final_intercept -> throwing
UAV: servo success -> ok_throw -> returning (without a GCS command)
GCS:                   ack
UAV: Init XY + yaw 0 -> ok_return -> downing -> ok_downing
GCS:                      ack                       ack
UAV: touchdown -> Disarm -> MANUAL -> ok_down
GCS:                                      ack
```

The B point is locked from the ARM origin and initial yaw `psi0`, while holding
the climb altitude and initial yaw:

```text
xB = x0 + 2.375*cos(psi0) + 0.375*sin(psi0)
yB = y0 + 2.375*sin(psi0) - 0.375*cos(psi0)
```

The planar distance is approximately `2.404 m`, or approximately `8.97 deg`
right of forward. Before the last `0.5 s` prediction window, the UAV shapes
translation toward the nearest relative `0/90/180/270 deg` direction without
turning its nose. In the final window it removes this angle constraint and
directly intercepts the predicted target. Payload release requires the first
predicted relative distance at or below `0.20 m`.
