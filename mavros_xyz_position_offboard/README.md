# ROS 2 MAVROS 原生 XYZ 位置节点

这是独立的新工具，目标环境为 PX4 FMUv6C.x、PX4 1.17.0、Raspberry Pi 5、
ROS 2 Jazzy、MAVROS 2.14.0 和 MTF02P 光流＋测距。它不修改、导入或替换
`tools/mavros_height_offboard`；旧工具仍是 `PositionTarget` 的“XY 位置＋Z 加速度”
历史路线，本目录改用 PX4 原生 XYZ 位置闭环。

构建、测试、控制授权和运行步骤记录于
[C++17 操作记录](docs/cpp17_operation_record.md)。

## 默认行为

`mavros_xyz_position_node` 是 C++17 `ament_cmake` 可执行文件，默认只创建订阅并输出缩进清晰的终端摘要：

- 自动将 LCP 的 `lcp_nwu` XY+yaw 桥接到 `/mavros/vision_pose/pose_cov`：固定转换为 ROS ENU，且仅接受新鲜 `STATUS=2`；
- 不创建 `/mavros/setpoint_raw/local` publisher；
- 不创建 `/mavros/set_mode` client；
- 不创建 `/mavros/cmd/arming` client；
- 不写 PX4 或 MAVROS 参数；
- 没有 force-arm/force-disarm 路径。

默认同时在当前工作目录的 `artifacts/` 下创建
`mavros-xyz-flight-<UTC>-<pid>.log`，内容与终端摘要一致。使用 `--output jsonl`
时改为创建同名 `.jsonl` 文件，每条记录包含完整遥测，以及便于检索的
`flight_snapshot`：实际 XYZ、实际 XYZ 速度、XYZ 位置目标、目标高度、电压/电量、
range/光流年龄、模式和状态阶段。可用 `--artifact-dir` 修改日志目录；`--output
summary` 为默认的人类可读格式；无论 artifact 选择哪种格式，终端始终输出该摘要。

range、optical-flow 和 FCU URL 都必须作为参数提供，节点不会把旧 README 的设备路径
或传感器身份套到当前硬件。当前只读核验看到的话题是
`/mavros/px4flow/ground_distance` 和
`/mavros/px4flow/raw/optical_flow_rad`，但更换 launch/plugin 配置后必须重新核验。

LCP 桥接默认启用，使用 XY/yaw 标准差 `0.20`、`lcp_nwu -> lcp_enu` 变换。它不读取
PX4 当前 yaw，也不设定 PX4 参数。仅为隔离诊断可加
`--disable-lcp-vision-bridge`；常规运行不应添加该选项。Python 旧桥接不能与本节点同时运行。

## 原生位置控制

MAVROS 2.14 `setpoint_raw` 接口订阅 `PositionTarget` 的 local setpoint，向飞控发送
XYZ＋yaw，并通过 type mask 忽略速度、加速度和 yaw-rate。节点保留 ROS ENU 位置值，
设定点明确标记 `FRAME_LOCAL_NED`；启用 publisher 前还必须确认 MAVROS 的
`mav_frame=LOCAL_NED`；`BODY_NED/BODY_OFFSET_NED` 不满足固定世界 XY 的设计。

节点在全部门禁通过后锁存一次当前 X/Y/Z 和规范化姿态：

- warmup 期间持续发布原始锁存位置；
- 起飞、定高悬停、航点和降落阶段固定 X/Y，并从首条设定点开始保持 LCP NWU yaw=0（正北；ROS ENU yaw=+π/2）；
- heartbeat 确认正常 ARM 后，Z 才开始向 `z0 + relative_z` 移动；
- 爬升阶段不因 LCP `STATUS=3` 停止；定点定高保持 10 秒后，必须等到新鲜
  `STATUS=2` 和 `/lcp/odometry`，然后保持北向目标（ROS ENU yaw=+π/2）才开始航点运动；
- Z 使用速度、加速度有界的五次轨迹；
- 定高悬停完成后，按锁存航向执行四段 0.5 m 航点：前、左、后、右；
- 航点最后回到初始 XY，再执行下降和正常解锁；
- 默认 20 Hz、warmup 2 秒。

“前/左/后/右”按 ARM 时锁存的机体 yaw 在 ROS ENU local XY 中计算。XY 航点同样使用
速度、加速度有界的五次轨迹，不直接跳变位置设定值。默认有界飞行时间为 60 秒，
可用 `--waypoint-leg`、`--waypoint-max-speed`、`--waypoint-max-accel` 和
`--waypoint-tolerance` 调整。

## 三层显式确认

以下每组必须完整，残缺组合会在 ROS 初始化和资源创建之前退出。

1. PositionTarget publisher：
   `--enable-position-setpoints`、`--ack-native-xyz-position-control`、
   `--ack-setpoint-streaming-risk`、
   `--confirm-setpoint-mav-frame-local-ned`、
   `--confirm-range-source`、`--confirm-optical-flow-source`。
2. SetMode client：第一组加
   `--request-offboard-mode`、`--ack-disarmed-mode-switch`。
3. CommandBool client：前两组加
   `--execute-bounded-flight`、`--ack-normal-arm-only`、
   `--ack-propeller-configuration-safe`、
   `--ack-area-and-personnel-clear`、
   `--ack-independent-emergency-stop-ready`、
   `--ack-valid-flight-battery-installed`、
   `--ack-direct-px4-xy-fusion-evidence`，并提供可审计的
   `--px4-xy-fusion-evidence-label`。

若确实需要允许地面附近读数低于 `sensor_msgs/Range.min_range`，可显式加入
`--ignore-declared-min-range`；进入有界飞行控制时还必须加入
`--ack-range-below-declared-min`。该开关只改变本节点的软件下限，不修改传感器声明、
PX4 参数或传感器物理有效范围；本节点配置的 `--configured-min-range`、最大距离、
新鲜度和跳变保护仍然生效。

服务 response 只记录，不当成状态成功；只有后续 `/mavros/state` heartbeat 实际回报
`OFFBOARD` 或 `armed=true` 才推进状态机。ARM 后模式丢失不会抢回 OFFBOARD；节点进入
abort/landing 路径，必要时只请求正常 `AUTO.LAND`。只有落地、回到 Z 基线并正常解锁后
才会给出 `PASS` 或 `ABORTED_SAFE`。

## 预检门禁

控制候选要求以下 MAVROS 可见证据全部新鲜、有限且通过：

- `/mavros/state`：连接、heartbeat 新鲜、未解锁、PX4 `STANDBY`；
- `/mavros/sys_status`：`MAV_SYS_STATUS_PREARM_CHECK` health 位为 1，且
  `enabled & ~health == 0`；
- `/mavros/battery`：电池存在、有效电压和剩余比例高于阈值；
- `/mavros/extended_state`：`ON_GROUND`；
- local pose/velocity：XYZ、四元数和速度有效、新鲜，静止速度不过限；
- `/mavros/estimator_status`：attitude、水平/垂直速度、水平位置和垂直位置
  legacy status flags 有效，GPS glitch/accel error 未置位；
- Range：来源已确认、消息新鲜、数值落在配置范围内，并默认落在消息声明 min/max 内；
  只有显式 `--ignore-declared-min-range` 时才忽略声明的最小值，声明的最大值仍参与检查；
- OpticalFlowRad：来源已确认、消息新鲜、integration time 有效、积分角有限、
  quality 达标。

OpticalFlowRad `temperature` 会写入 JSONL，但在来源语义另行确认前不参与门禁。
地面静止时 `const_pos_mode_status_flag=true` 可能来自 PX4 的 `vehicle_at_rest`，因此它
会被记录但不单独阻断地面预检；ARM/飞行阶段出现该位则立即中止。

当前实测 `range=0.05 m`、消息声明 `min_range≈0.30 m`。默认策略会报
`outside declared/accepted interval`；启用显式覆盖后可通过本节点的软件下限，但这不代表
`0.05 m` 已成为传感器有效测量。当前 `battery=0 V` 仍必须阻断。原始 flow
`quality=59` 不能抵消电池、XY 原生融合或传感器有效性问题，也不能单独证明 EKF 已融合光流。

## MAVROS 可见性边界

MAVROS 2.14 的这些标准话题不能直接证明：

- PX4 `vehicle_local_position.xy_valid/z_valid/v_xy_valid/v_z_valid/dead_reckoning`；
- `estimator_status_flags.cs_opt_flow/cs_rng_hgt/reject_*`；
- `estimator_aid_src_*` 的 `fused`、`innovation_rejected`、test ratio 和年龄；
- `sensor_optical_flow.distance_available`；
- `sensor_msgs/Range` 背后的 MAVLink `DISTANCE_SENSOR.orientation`。

节点不会伪造这些字段。`EstimatorStatus` 门禁只是必要条件，不足以批准自由带桨 XYZ
悬停；飞行确认组额外要求一份 PX4 shell/ULog 原生融合证据标签。若无法取得，保持
只读验证或使用物理约束架。

## 本地测试

```bash
cd /home/pi/px4-test-tools
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mavros_xyz_position_offboard
colcon test --packages-select mavros_xyz_position_offboard
source install/setup.bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node --help
```
