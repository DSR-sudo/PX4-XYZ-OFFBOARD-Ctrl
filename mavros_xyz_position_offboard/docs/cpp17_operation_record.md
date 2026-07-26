# C++17 ROS 2 包操作记录

本文记录 `mavros_xyz_position_offboard` 从 Python 迁移为 C++17 `ament_cmake` 包后的日常操作、验证方式和职责边界。

## 目录与职责

| 路径 | 职责 |
| --- | --- |
| `include/.../common`、`src/common` | 配置、CLI、遥测值类型、数值函数与 JSONL/摘要日志。 |
| `include/.../initialization`、`src/initialization` | MAVROS/LCP 订阅、测距保护、预检和飞行期门禁、LCP 初始化客户端。 |
| `include/.../navigation`、`src/navigation` | 初始位姿锁存、受速度/加速度约束的五次 XYZ 轨迹、航点和保持策略。 |
| `include/.../offboard`、`src/offboard` | `PositionTarget` 发布、模式/解锁异步请求、飞行状态机、LCP-hold 与安全降落。 |
| `src/main.cpp` | 剥离 ROS 参数、解析应用 CLI，并通过 `SingleThreadedExecutor` 启动唯一节点。 |
| `test/test_lcp_navigation.cpp` | LCP 门禁、轨迹约束、`PositionTarget` 映射和 CLI 门禁测试。 |

`MavrosNativeXYZNode` 是唯一 ROS 节点，节点名为 `mavros_native_xyz_position`。Navigation 与 OFFBOARD 之间只传递内部 C++ `PositionSetpoint`，不会创建内部 ROS 话题。

## 构建

在工作空间根目录执行：

```bash
cd /home/pi/px4-test-tools
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mavros_xyz_position_offboard
source install/setup.bash
```

构建完成后，可确认安装入口：

```bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node --help
```

## 单元测试

```bash
cd /home/pi/px4-test-tools
source /opt/ros/jazzy/setup.bash
colcon test --packages-select mavros_xyz_position_offboard
colcon test-result --verbose
```

当前测试覆盖：

- LCP 服务初始化之后必须收到新的 `STATUS=2` 和里程计样本；
- LCP 失败或遥测过期不能通过门禁；
- LCP-hold 使用本地位置冻结，yaw=0 不使用 LCP 坐标；
- XYZ 五次轨迹不超过配置速度和加速度；
- `PositionTarget` 使用 `FRAME_LOCAL_NED`，忽略速度、加速度和 yaw-rate；
- 默认 CLI 约束保持不变，危险控制的残缺确认组合会被拒绝。

## 只读监视运行

以下命令只创建订阅、输出状态和写入 artifact 日志；不会创建设定点发布器、模式客户端或解锁客户端：

```bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node \
  --confirmed-fcu-url udp://127.0.0.1:14540 \
  --range-topic /mavros/px4flow/ground_distance \
  --range-source-label downward_range_confirmed \
  --optical-flow-topic /mavros/px4flow/raw/optical_flow_rad \
  --optical-flow-source-label optical_flow_confirmed
```

日志默认写入当前工作目录的 `artifacts/`。使用 `--output jsonl` 可切换为每行一个严格 JSON 记录；使用 `--artifact-dir DIR` 可改变输出目录。

## 控制授权顺序

控制能力由 CLI 显式确认逐级开启，缺少任一确认参数会在创建 ROS 资源前以错误退出。

1. 发布 `PositionTarget`：
   `--enable-position-setpoints`、`--ack-native-xyz-position-control`、
   `--ack-setpoint-streaming-risk`、`--confirm-setpoint-mav-frame-local-ned`、
   `--confirm-range-source`、`--confirm-optical-flow-source`。
2. 请求 `OFFBOARD`：第一组基础上增加
   `--request-offboard-mode`、`--ack-disarmed-mode-switch`。
3. 有界飞行与普通解锁：第二组基础上增加
   `--execute-bounded-flight`、`--ack-normal-arm-only`、
   `--ack-propeller-configuration-safe`、`--ack-area-and-personnel-clear`、
   `--ack-independent-emergency-stop-ready`、
   `--ack-valid-flight-battery-installed`、
   `--ack-direct-px4-xy-fusion-evidence`，并提供
   `--px4-xy-fusion-evidence-label LABEL`。

服务应答本身不推进飞行状态机。节点仅在之后的 `/mavros/state` 心跳确实报告 `OFFBOARD` 或 `armed=true` 时继续执行。

## 运行动作与安全策略

- wall timer 默认以 20 Hz 工作；`--publish-rate` 仅允许 10–50 Hz。
- LCP 初始化由 Initialization 发起。初始化服务接受后，仍须有足量的新鲜 `STATUS=2` 和里程计样本才能解锁预检门禁。
- OFFBOARD 是唯一发布 `/mavros/setpoint_raw/local`、请求 `/mavros/set_mode` 和 `/mavros/cmd/arming` 的组件。
- 五次轨迹约束 Z 与 XY 的设定点速度/加速度；到达容差后才转换航点。
- 航点阶段 LCP 失效会进入 `lcp_hold`，冻结本地设定点；超过 `--lcp-unhealthy-hold-timeout` 后进入安全降落。
- 飞行期出现阻断门禁、模式丢失或最大飞行时间超限时，节点保持 XY、下降至锁存 Z 基线，并仅使用正常的 `AUTO.LAND`/上锁路径。

## 常用参数校验

可用下列命令验证参数拒绝路径，不会创建节点：

```bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node \
  --confirmed-fcu-url udp://127.0.0.1:14540 \
  --range-topic /range --range-source-label downward \
  --optical-flow-topic /flow --optical-flow-source-label flow \
  --publish-rate 5
```

该命令应以错误退出，因为设定点发布频率必须位于 10–50 Hz。单独提供任何危险控制确认（例如仅提供 `--enable-position-setpoints`）同样会被拒绝并列出缺失确认项。

## 迁移后验证记录

在 ROS 2 Jazzy 与 MAVROS 2.14 环境中已执行：

```bash
colcon build --packages-select mavros_xyz_position_offboard
colcon test --packages-select mavros_xyz_position_offboard
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node --help
```

构建和 7 个 gtest 均通过。此验证未连接真实飞控；连接飞控前仍应按现场飞行安全流程重新确认传感器来源、飞控 URL、空间隔离和 PX4 原生融合证据。
