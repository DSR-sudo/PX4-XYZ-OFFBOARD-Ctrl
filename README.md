# PX4 / MAVROS 飞行测试工具

本仓库用于 PX4 飞控、MAVROS、雷达 LCP 建系、测距和光流融合的只读核验与受限
XYZ 位置测试。核心 ROS 2 包是 `mavros_xyz_position_offboard`，另有 Python
桥接脚本、参数读取工具、操作记录和测试产物。

默认目标环境记录为 PX4 FMUv6C.x、PX4 1.17.0、Raspberry Pi 5、ROS 2 Jazzy、
MAVROS 2.14.0 和 MTF02P 光流/测距。实际运行前必须重新确认当前硬件、话题来源、
PX4 参数和坐标系；README 中的设备型号不构成自动识别结果。

## 核心能力

`mavros_xyz_position_node` 是一个 C++17 ROS 2 节点，按显式授权逐级创建控制资源：

- 默认只订阅 MAVROS、传感器和 LCP 话题，执行预检并输出状态，不创建设定点发布器。
- 满足完整确认组后，才创建 `/mavros/setpoint_raw/local` 发布器。
- 再满足模式确认后，才创建 `/mavros/set_mode` 客户端。
- 最后满足受限飞行确认后，才创建 `/mavros/cmd/arming` 客户端。
- 不写入 PX4/MAVROS 参数，不使用 force-arm/force-disarm，不把服务响应当成飞控状态成功。
- 所有模式和解锁结果必须等待后续 `/mavros/state` heartbeat 实际确认。

## 内部架构

```text
命令行参数与确认门禁
          │
          ▼
MavrosNativeXYZNode（20 Hz 单线程定时器）
   ┌──────┼──────────────┬──────────────┐
   ▼      ▼              ▼              ▼
Initialization  LcpVisionBridge  Navigation  ArtifactLogger
   │              │              │
   │ 订阅遥测     │ LCP NWU→ENU  │ 五次轨迹
   │ 预检门禁     │ 外部视觉发布  │ 方形航点
   │ RangeGuard   │              │ XYZ 设定点
   └──────────────┴──────────────┴──────────────▶ OFFBOARD 状态机
                                                        │
                                                        ▼
                                      PositionTarget / SetMode / CommandBool
```

`Initialization` 只负责接收并缓存遥测、判断新鲜度和门禁；`Navigation` 是不依赖 ROS
消息的轨迹规划器；`MavrosNativeXYZNode` 负责把两者串成预检、飞行和降落状态机。
控制时间使用单调时钟，避免 ROS 时间跳变影响超时判断。

## 输入话题与输出接口

默认输入话题如下，均可通过命令行选项修改：

| 类别 | 默认话题 | 用途 |
| --- | --- | --- |
| 飞控 | `/mavros/state` | 连接、模式、解锁和 heartbeat |
| 飞控 | `/mavros/sys_status` | 传感器 enabled/health 位掩码 |
| 飞控 | `/mavros/battery` | 电池存在性、电压和电量 |
| 飞控 | `/mavros/extended_state` | `ON_GROUND` 状态 |
| 飞控 | `/mavros/local_position/pose` | 本地 XYZ 与姿态 |
| 飞控 | `/mavros/local_position/velocity_local` | 本地 XYZ 速度 |
| 飞控 | `/mavros/estimator_status` | 姿态、速度、位置和故障标志 |
| 传感器 | 命令行指定 | `sensor_msgs/Range` 测距 |
| 传感器 | 命令行指定 | `mavros_msgs/OpticalFlowRad` 光流 |
| LCP | `/lcp/status` | LCP 状态，只有新鲜 `2` 才表示地图锁定 |
| LCP | `/lcp/odometry` | LCP 平面位置和 yaw |
| 输出 | `/mavros/vision_pose/pose_cov` | LCP 锁定后的外部视觉位姿 |
| 输出 | `/mavros/setpoint_raw/local` | 启用控制后的 XYZ+yaw 位置设定点 |

LCP 初始化服务默认为 `/lcp/start_initialization`。节点会在 LCP 服务请求前锁存
状态/里程计序列号基线，旧的 `STATUS=2` 消息不能为新的初始化请求“充数”。默认至少
需要 3 个初始化基线之后的新鲜健康样本。

## LCP 视觉桥接

LCP 驱动在 `lcp_nwu` 中定义 `+X=北、+Y=西/左、+Z=上`，yaw=0 指向北。内置
`LcpVisionBridge` 只接受新鲜 `STATUS=2` 且 frame 为 `lcp_nwu` 的里程计，然后整体
转换为 ROS ENU：

```text
lcp_nwu:  (x=north, y=west, yaw=0 north)
       ↓  x_enu=-y_nwu, y_enu=x_nwu, yaw_enu=yaw_nwu+π/2
lcp_enu:  (x=east, y=north)
       ↓
/mavros/vision_pose/pose_cov
```

XY 和 yaw 使用可配置的标准差，Z、roll、pitch 使用大协方差，因为该桥接只为 XY/yaw
外部视觉输入服务。它不会启用 PX4 EKF 融合，也不会修改任何 PX4 参数。需要隔离诊断
时可使用 `--disable-lcp-vision-bridge`；通常应保持默认启用。

## 飞行状态机

当只读监视模式运行时，节点停留在预检报告阶段。启用完整控制后，流程为：

```text
预检
  → 请求 LCP 建系
  → 等待新鲜 STATUS=2 和里程计样本
  → 锁存当前 XYZ，固定北向 yaw
  → 设定点预热（默认 2 s）
  → 请求 OFFBOARD（若已授权）
  → 请求普通 ARM（若已授权）
  → 爬升到 origin_z + relative_z
  → 定高悬停（默认 10 s）
  → 前 / 左 / 后 / 右四段方形航点
  → 回到初始 XY
  → 下降
  → 落地确认后普通解锁
  → PASS 或 ABORTED_SAFE
```

`Navigation` 为 Z 和 XY 都使用五次多项式轨迹，并通过拉长轨迹时间限制速度和加速度。
航点以 ARM 时的固定航向在 ROS ENU local XY 中生成，默认每边 0.5 m；不直接跳变位置
设定点。

LCP 失效策略与阶段有关：爬升阶段不因 LCP `STATUS=3` 停止；定高悬停结束前必须等到
新鲜 `STATUS=2` 才进入航点；航点期间 LCP 失效会冻结 XY/Z 进入 `lcp_hold`，恢复则
继续，超过保持超时则进入安全降落。其他阻断条件（掉线、模式丢失、漂移、估计器故障、
电池或传感器超时等）会固定水平位置并进入降落路径；必要时只请求正常 `AUTO.LAND`。

## 预检门禁

预检会同时检查：

- MAVROS 已连接，heartbeat、SYS_STATUS 和落地状态新鲜；飞行器未解锁并处于地面状态。
- PX4 enabled-but-unhealthy 传感器掩码没有不可接受故障。
- 电池存在，电压默认不低于 14 V，地面预检电量默认不低于 30%。
- local pose、local velocity、四元数有限且新鲜，地面速度不过限。
- MAVROS 暴露的估计器姿态、速度、水平/垂直位置标志有效，GPS glitch 和 accel error 未置位。
- 测距来源和光流来源通过命令行显式确认；消息新鲜，光流积分时间有效，质量默认至少 20。
- local Z 与向下测距的差值不超过 0.30 m。
- 测距值处于配置范围内，并对短时间大跳变进行故障锁存和连续稳定样本恢复。

RangeGuard 默认使用配置下限/上限 `0.02..12.0 m`，默认可忽略消息声明的最小距离，
但仍检查声明的最大距离。使用 `--enforce-declared-min-range` 可强制同时遵守消息声明
的最小距离；`--ack-range-below-declared-min` 只记录操作者对低于声明下限场景的审计确认，
不会改变传感器物理有效范围。

MAVROS 话题不能直接证明 PX4 内部所有 `vehicle_local_position`、`estimator_aid_src_*`
或传感器融合字段。程序不会伪造这些字段；执行真实带桨飞行仍需要独立的 PX4 shell/ULog
融合证据和物理安全措施。

## 三层显式确认

### 1. 创建位置设定点发布器

必须同时提供：

```text
--enable-position-setpoints
--ack-native-xyz-position-control
--ack-setpoint-streaming-risk
--confirm-setpoint-mav-frame-local-ned
--confirm-range-source
--confirm-optical-flow-source
```

设定点使用 `mavros_msgs/PositionTarget` 的 `FRAME_LOCAL_NED`，保留 PX/PY/PZ/YAW，
忽略速度、加速度和 yaw-rate。程序在内部维护 ROS ENU 值，并要求操作者确认 MAVROS
的 `mav_frame=LOCAL_NED` 语义。

### 2. 创建模式客户端

在第一组确认之外，再提供：

```text
--request-offboard-mode
--ack-disarmed-mode-switch
```

程序才会创建 `/mavros/set_mode` 客户端并周期性请求 `OFFBOARD`；服务返回不等于模式已经
切换，必须等 `/mavros/state` 回报 `OFFBOARD`。

### 3. 创建解锁客户端和受限飞行

在前两组确认之外，再提供：

```text
--execute-bounded-flight
--ack-normal-arm-only
--ack-propeller-configuration-safe
--ack-area-and-personnel-clear
--ack-independent-emergency-stop-ready
--ack-valid-flight-battery-installed
--ack-direct-px4-xy-fusion-evidence
--px4-xy-fusion-evidence-label "可审计的证据标签"
```

程序只发送普通 `CommandBool` ARM/解锁请求。若任一组确认不完整，参数解析阶段会报错，
不会创建对应资源。

## 构建与测试

```bash
cd /home/pi/px4-test-tools
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mavros_xyz_position_offboard
colcon test --packages-select mavros_xyz_position_offboard
colcon test-result --verbose
source install/setup.bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node --help
```

单元测试覆盖 LCP 新鲜样本基线、LCP→ENU 变换、五次轨迹速度/加速度约束、LOCAL_NED
设定点映射和不完整确认拒绝。

## 运行示例

先启动 MAVROS，并把 `fcu_url`、测距和光流话题改成当前实机经过核验的值：

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch mavros px4.launch \
  fcu_url:=/dev/serial/by-id/你的飞控设备:2000000 \
  gcs_url:=udp://127.0.0.1:14551@
```

只读核验/摘要模式：

```bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node \
  --confirmed-fcu-url /dev/serial/by-id/你的飞控设备:2000000 \
  --range-topic /mavros/px4flow/ground_distance \
  --range-source-label MTF02P-ground-distance \
  --optical-flow-topic /mavros/px4flow/raw/optical_flow_rad \
  --optical-flow-source-label MTF02P-optical-flow
```

仓库中的 `command.txt` 保存了一条带完整确认项的有界测试命令。执行前必须重新检查
飞控连接、传感器方向、螺旋桨/区域安全、PX4 XY 融合证据和独立急停，不应直接照抄设备路径。

## 日志与产物

节点默认在当前目录 `artifacts/` 创建 `mavros-xyz-flight-<UTC>-<pid>.log`，终端输出
为人类可读摘要。指定 `--output jsonl` 后生成同名 `.jsonl` 文件，每行包含：

- 当前阶段、结果、错误和安全中止原因；
- 实测 XYZ、XYZ 速度、目标位置、目标高度和垂直速度；
- 飞控模式/解锁、电池、测距、光流、估计器和 LCP 遥测；
- LCP、SetMode、ARM 服务事件和所有审计配置。

`--artifact-dir` 可以指定日志目录。`artifacts/`、`build/`、`install/` 和 `log/` 已在
`.gitignore` 中排除，避免把运行产物提交进仓库。

## 目录结构

```text
px4-test-tools/
├── mavros_xyz_position_offboard/       ROS 2 C++17 节点、库、测试和包级 README
│   ├── src/common/                     CLI、类型、日志
│   ├── src/initialization/             遥测订阅、预检、RangeGuard、LCP 门禁
│   ├── src/navigation/                五次 XYZ 轨迹和方形航点
│   ├── src/bridge/                    LCP NWU→ENU 外部视觉桥
│   ├── src/offboard/                  OFFBOARD/ARM/降落状态机
│   └── test/                           C++ 单元测试
├── lcp_yaw_vision_bridge.py            独立 Python LCP→MAVROS 外部视觉桥
├── read_px4_params.py                  PX4 参数读取辅助工具
├── docs/                               操作记录、融合核验和迁移说明
├── command.txt                         经过审计的命令模板
└── artifacts/                          本地运行时生成，已忽略
```

独立 Python 桥接脚本与 C++ 节点的内置桥接不能同时运行，否则会向同一外部视觉话题
重复发布。
