# LCP → MAVROS C++ 桥接迁移（2026-07-27）

Python 文件 `lcp_yaw_vision_bridge.py` 的运行时职责已迁入
`mavros_xyz_position_offboard` ROS 2 包的 `LcpVisionBridge` C++ 类。它是
`mavros_xyz_position_node` 的成员，因此运行原有包节点时会自动启动，不需要另开 Python 终端。

## 固定行为

- 只接受新 LCP 源端发布的 `header.frame_id=lcp_nwu` 和新鲜 `STATUS=2`。
- `lcp_nwu`（北、西/左、上）完整转换为 ROS ENU：`east=-west`、`north=north`、`yaw_enu=pi/2+yaw_nwu`。
- 发布 `/mavros/vision_pose/pose_cov`，默认 XY/yaw 标准差均为 `0.20`，即方差 `0.04`；z、roll、pitch 方差为 `10000`。
- 没有 PX4 当前 yaw 的自动偏置，也不会设置 PX4 参数、发送设定点、请求 Offboard 或解锁。
- 任何非 `lcp_nwu` 帧、过期状态、无效位置或四元数都会被拒绝，不发布视觉消息。

## 启动

停止旧 Python 桥接后，按原有只读命令启动包节点即可自动桥接：

```bash
source /opt/ros/jazzy/setup.bash
source /home/pi/px4-test-tools/install/setup.bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node \
  --confirmed-fcu-url /dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:2000000 \
  --range-topic /mavros/px4flow/ground_distance \
  --range-source-label MTF02P-ground-distance \
  --optical-flow-topic /mavros/px4flow/raw/optical_flow_rad \
  --optical-flow-source-label MTF02P-optical-flow \
  --output summary
```

可选调节：`--lcp-vision-xy-stddev`、`--lcp-vision-yaw-stddev`、
`--lcp-vision-max-status-age`、`--lcp-vision-pose-topic`、`--lcp-vision-input-frame`。
`--disable-lcp-vision-bridge` 仅供隔离诊断；默认必须省略它。

切换时必须先停止 Python 桥接，再启动 C++ 节点，确保 `/mavros/vision_pose/pose_cov` 始终只有一个发布者。

## 地面切换验证

- 包编译成功，C++ 单元测试共 9 项全部通过；其中 `LcpVisionBridgeTest` 验证了 NWU→ENU 的 XY、yaw 与协方差。
- 已停止 Python 桥接，视觉话题发布者数量先确认降为 `0`，再启动 C++ 包节点。
- 切换后 `/mavros/vision_pose/pose_cov` 发布者数量为 `1`，节点为 `mavros_native_xyz_position`；没有 Python 发布者。
- 单次输出为 `frame_id=lcp_enu`，yaw 四元数 `z=w=0.70710678`，XY/yaw 方差均为 `0.04`，z/roll/pitch 方差为 `10000`。
- PX4 的 3 个 EKF 实例均保持 `cs_ev_pos=True`、`cs_ev_yaw=True`、`reject_hor_pos=False`、`reject_yaw=False`，并保持 `cs_mag=False`、`cs_gnss_pos=False`。
- C++ 节点使用默认只读模式，`phase=blocked`；未创建 setpoint publisher、未请求 Offboard、未解锁。
