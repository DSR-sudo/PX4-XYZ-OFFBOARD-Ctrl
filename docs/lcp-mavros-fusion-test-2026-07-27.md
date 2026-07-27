# LCP → MAVROS → PX4 地面融合测试记录（2026-07-27）

## 测试条件

- 飞控：`3D Robotics PX4 FMU v5.x`，PX4 `1.17.0`
- 飞控串口：`/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:2000000`
- MAVROS：已连接，`armed=false`，没有启动 Offboard 或解锁。
- LSLIDAR：成功打开 `/dev/ttyAMA0`，`/scan` 约 `10 Hz`。
- 设备处于不规则空间，不在四面矩形网罩/矩形房间内。
- 电池未连接，MAVROS 读数为 `0.000 V`，因此控制节点保持阻塞。

## 启动结果

启动顺序如下：

1. 复用并重启 MAVROS，确认 FCU heartbeat `connected=true`。
2. 启动 `ros2 launch lslidar_driver lsn10p_launch.py`。
3. 调用 `/lcp/start_initialization`，服务返回 `success=true`。
4. 启动 `lcp_yaw_vision_bridge.py`，XY 标准差参数为 `0.20 m`，yaw 标准差为 `0.20 rad`。
5. 启动 `mavros_xyz_position_node` 的只读监视模式。

## 观测结果

| 项目 | 结果 |
| --- | --- |
| MAVROS local XY | 约 `x=0.04 m, y=-0.03 m` |
| MAVROS local yaw | 约 `-2.32 rad`（`-133°`） |
| `/lcp/status` | 持续为 `3`（不健康） |
| `/lcp/odometry` | 无消息 |
| `/lcp/yaw` | 无消息 |
| `/mavros/vision_pose/pose_cov` | 无有效外部视觉输入 |
| `mavros_xyz_position_node` | `phase=blocked / result=UNCONFIRMED` |

## 结论

本次未形成 LCP XY/yaw 到 PX4 的实际融合。原因是当前 LCP 算法要求完整四面矩形墙体，使用四墙矩形拟合和锁定地图；不规则空间无法建立有效地图，因此安全门禁保持 `STATUS=3`，桥接程序不会发布外部视觉消息。该行为符合设计，不能通过绕过状态门禁来强行注入无效位置和航向。

MAVROS 的 local XY/yaw 读数是 PX4 当前本地估计值，不是 LCP 融合结果。当前没有 `cs_ev_pos=true` 或 `cs_ev_yaw=true` 的证据；在本测试中也没有 LCP 消息可供 PX4 融合。

## 本次代码调整

- `lcp_yaw_vision_bridge.py`：将 XY 协方差从原 yaw-only 的 `10000` 改为可配置值，测试使用 `0.20 m` 标准差；z/roll/pitch 仍保持高协方差，不参与融合。
- 修复 ROS 2 Jazzy 下桥接状态日志的 `get_logger().info()` 参数调用错误。
- 未修改 PX4 参数；前一阶段的 `EKF2_EV_CTRL=9`、`EKF2_GPS_CTRL=0`、`EKF2_MAG_TYPE=5` 保持不变。

## 安全状态

全程未启动 Offboard、未发送位置设定点、未解锁、未给电机供电。下一次测试必须在 LCP 能够达到 `STATUS=2` 的规则四墙环境中进行；若目标是支持不规则空间，需要先重新设计和验证 LCP 的地图/定位算法，不能仅靠 PX4 参数改变实现。

## 网罩内复测：融合成功

设备移入规则网罩后重新上电，重新启动 MAVROS、`lsn10p_launch.py` 和
`lcp_yaw_vision_bridge.py`，并再次调用 `/lcp/start_initialization`。

### 输入链路

| 项目 | 实测结果 |
| --- | --- |
| `/lcp/status` | `2`（锁定/健康） |
| `/lcp/odometry` | 持续发布；静止时 XY 接近 `(0, 0) m` |
| `/lcp/yaw` | 约 `-1.431 rad` |
| bridge yaw 偏置 | `-2.6405 rad`，在 connected、disarmed、ON_GROUND 时锁定 |
| `/mavros/vision_pose/pose_cov` | 持续发布 LCP 对齐后的 XY+yaw；XY/yaw 方差均为 `0.04` |
| MAVROS local pose | 静止时 XY 接近 `(0, 0) m`，与 LCP 对齐后的外部视觉值一致 |

### PX4 融合证据

通过 MAVROS 本机 GCS UDP 端口转发只读 NSH 请求读取
`estimator_status_flags`。全部 3 个 EKF 实例均满足：

- `cs_ev_pos: True`
- `cs_ev_yaw: True`
- `cs_gnss_pos: False`、`cs_gps_hgt: False`
- `cs_mag: False`、`cs_mag_hdg: False`
- `reject_hor_pos: False`、`reject_yaw: False`
- `cs_ev_yaw_fault: False`

这证明 PX4 正在融合 LCP 的水平 XY 和 yaw，而不是只接收 MAVROS 消息。

### 地面安全状态

- 飞控保持 `armed=false`、`AUTO.LOITER`；未请求 Offboard，也没有发送位置设定点。
- `mavros_xyz_position_node` 只读运行；其 LCP 采样持续增加，状态为 `2`。
- 电池遥测约 `16.56 V`，但本轮未做任何电机或飞行测试。
