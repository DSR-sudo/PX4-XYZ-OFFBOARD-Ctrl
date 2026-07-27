# PX4 LCP 参数改写记录（2026-07-27）

## 操作范围

- 目标设备：`3D Robotics PX4 FMU v5.x`
- PX4：`Release 1.17.0`，硬件架构 `PX4_FMU_V5`
- 操作前 MAVROS 已关闭，螺旋桨不转，飞控未解锁。
- 串口：`/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00`
- 目的：关闭 PX4 的 GPS EKF 融合，启用 LCP 外部视觉的水平 XY 与 yaw 融合。

## 参数变更

| 参数 | 改写前 | 改写后 | 作用 |
| --- | ---: | ---: | --- |
| `EKF2_EV_CTRL` | `0` | `9` | 启用外部视觉水平位置（bit 0，XY）和 yaw（bit 3）；`1 + 8 = 9` |
| `EKF2_GPS_CTRL` | `7` | `0` | 禁止 EKF2 使用 GPS 的位置、速度和高度数据 |
| `EKF2_MAG_TYPE` | `0` | `5` | 不融合磁力计，使外部视觉 yaw 成为航向来源 |

以上参数已执行 `param save`，并在重启后再次确认带有 `+` 标记，表示已保存。

## 相关参数核验（未改写）

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `EKF2_EV_QMIN` | `0` | 保留当前外部视觉质量门限；适配当前 MAVROS 视觉消息的 `quality=0` |
| `EKF2_HGT_REF` | `2` | 保持现有测距高度参考配置 |
| `EKF2_BARO_CTRL` | `1` | 保持气压计融合 |
| `EKF2_RNG_CTRL` | `2` | 保持测距条件融合 |
| `EKF2_OF_CTRL` | `1` | 保持光流融合 |
| `COM_ARM_WO_GPS` | `1` | 允许无 GPS 解锁；仍须通过其余安全检查 |
| `SYS_HAS_GPS` | `1` | 未改写；表示硬件存在性，不是 GPS EKF 融合开关 |
| `SYS_HAS_MAG` | `1` | 未改写；磁力计硬件存在，但已由 `EKF2_MAG_TYPE=5` 禁止融合 |

## 重启后状态

重启后确认：

- `EKF2_EV_CTRL=9`、`EKF2_GPS_CTRL=0`、`EKF2_MAG_TYPE=5`，均为已保存值。
- `cs_gnss_pos=false`、`cs_gps_hgt=false`、`cs_mag=false`。
- MAVROS 和 LCP 当前未启动，因此 `cs_ev_pos=false`、`cs_ev_yaw=false` 是预期状态；这不代表外部视觉链路失败。
- 飞控仍为 `Disarmed`、`Hold`、`in failsafe: no`。

## 后续使用限制

当前 `lcp_yaw_vision_bridge.py` 将 XY 协方差设为 `10000`，原设计是 yaw-only 台架试验；因此本次只完成了 PX4 参数侧的 XY+yaw 融合准备，不能据此宣称当前桥接程序已经实际融合 LCP XY。启动 LCP/MAVROS 前，应先把桥接消息的 XY 协方差和坐标变换按实测配置完成，并在地面验证 `estimator_status_flags` 中的 `cs_ev_pos=true`、`cs_ev_yaw=true` 后再进行任何飞行测试。
