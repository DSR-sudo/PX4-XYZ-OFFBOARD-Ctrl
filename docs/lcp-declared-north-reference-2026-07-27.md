# LCP 人工北向参考实现记录（2026-07-27）

## 目标

设备没有使用磁力计或 GPS，但操作员能够在建系前把机体正前方可靠地摆向真实北方。目标是让 LCP 与 PX4 使用同一北向定义，而不是把矩形网罩的任意墙边当作 `0 rad`。

这不是传感器估计出的地磁北；它是一次受控摆放产生的人工北向参考。若机体或雷达在建系后发生转动、滑移，或雷达 +X 与机头不平行，误差会直接成为 XY 和 yaw 的共同偏差。

## 源端改动

修改了 `/home/pi/LSLIDARN10P` 中的 LCP 源码，而非仅在 MAVROS 端伪造一个 yaw 偏置：

- 新参数 `lcp_initial_heading_is_north`，默认 `true`，已写入 `lsn10p.yaml`。
- LCP 内部仍保持原有的墙体坐标用于矩形匹配；不会在飞行中切换/重建地图。
- 在 `STATUS=2` 锁图首帧，保存内部墙体 yaw，并将公开的 XY 和 yaw 以同一旋转转换到 `lcp_nwu`。
- `lcp_nwu` 为右手坐标：`+X=北`，`+Y=西/机体左侧`，`+Z=上`。建系首帧为 `(x=0, y=0, yaw=0)`；左转为正 yaw。
- `/lcp/odometry` 与 `/lcp/debug` 的 `header.frame_id` 在该模式下为 `lcp_nwu`；`/lcp/yaw` 同时输出这个北向 yaw。
- 增加 LCP 单元测试，验证初始 yaw 归零，并验证 XY 与 yaw 使用同一个旋转。

## MAVROS 融合链路

桥接脚本仍然必要，但不再标定或保存“PX4 当前 yaw − LCP yaw”偏置。它只检查输入帧必须为 `lcp_nwu`，再执行固定坐标变换：

```text
LCP NWU: (north, west, yaw_nwu)
  -> ROS ENU: (east=-west, north, yaw_enu=pi/2+yaw_nwu)
  -> MAVROS
  -> PX4 NED
```

所以建系时 `/lcp/yaw=0`，桥接后的 ROS ENU yaw 是 `+pi/2`，MAVROS 转到 PX4 NED 后正好是“机头朝北”的 `0 rad`。

## 每次建系流程

1. 保持飞控未解锁、ON_GROUND；确认雷达 +X 与机头正前方平行。
2. 将机头正前方摆到真实北方；此后不要移动机体或雷达安装位置。
3. 启动 MAVROS、LCP 和桥接；桥接必须使用新版，输入帧为 `lcp_nwu`。
4. 调用 `/lcp/start_initialization`，等待 `/lcp/status=2`。
5. 确认 `/lcp/yaw` 接近 `0 rad`、`/lcp/odometry.header.frame_id=lcp_nwu`，并确认桥接消息的 `frame_id=lcp_enu`、yaw 接近 `+1.571 rad`。
6. 仅在这些地面读数稳定后，再确认 PX4 `cs_ev_pos=true`、`cs_ev_yaw=true`。本次不解锁、不进入 Offboard。

如果需要恢复旧的任意墙体坐标，可将 `lcp_initial_heading_is_north` 设为 `false`；此时输出帧为 `lcp_map`，新版桥接会拒绝该输入以防把错误方向送入 PX4。

## 北向地面复测结果

在操作员确认机头和雷达 +X 已摆向北方后，停止旧桥接和旧 LCP 节点，重新启动已编译的 LCP 与新版桥接，并调用一次 `/lcp/start_initialization`。服务返回 `success=true`。

一次 `/lcp/debug` 采样：

```text
frame_id: lcp_nwu
status: 2
map_locked: true
pose_valid: true
position_x_m: -0.00035
position_y_m: -0.00097
yaw_rad: 0.0
front/rear: 2.058 / 2.107 m
left/right: 2.199 / 2.899 m
```

同轮 `/lcp/odometry` 也为 `frame_id=lcp_nwu`、单位四元数，`/lcp/yaw=0.0`。桥接后的单次 `/mavros/vision_pose/pose_cov` 为 `frame_id=lcp_enu`，四元数 `z=w=0.70710678`，即 ROS ENU yaw `+pi/2`，与“机头朝北”一致。

通过 MAVROS 的本机 GCS UDP 端口只读读取 `estimator_status_flags`；3 个 EKF 实例均为：

- `cs_ev_pos=True`、`cs_ev_yaw=True`
- `cs_gnss_pos=False`、`cs_gps_hgt=False`
- `cs_mag=False`、`cs_mag_hdg=False`
- `reject_hor_pos=False`、`reject_yaw=False`、`cs_ev_yaw_fault=False`

复测全程保持 `armed=false`、`AUTO.LOITER`、`ON_GROUND`；没有请求 Offboard，也没有发送控制设定点。
