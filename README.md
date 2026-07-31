# PX4 / MAVROS 飞行测试工具

本工作区用于 PX4 飞控、MAVROS、LCP 建图定位、向下测距、光流和地面站 UDP 协同任务的
受限测试。核心 ROS 2 C++17 包为 `mavros_xyz_position_offboard`；它实现 UAV 端任务状态机，
不修改 PX4/MAVROS 参数，也不使用 force-arm 或 force-disarm。

默认目标环境记录为 PX4 FMUv6C.x、PX4 1.17.0、Raspberry Pi 5、ROS 2 Jazzy、MAVROS
2.14.0 和 MTF02P 光流/测距。实际运行前必须重新确认硬件、话题来源、PX4 参数和坐标系；
本文档中的设备型号和命令不构成自动识别或飞行授权。

Plan1 B 点拦截协议位于
[`docs/plan1_intercept_udp_protocol.md`](docs/plan1_intercept_udp_protocol.md)。该版本不提供旧的
右移、GCS 前飞放行、匹配确认或返航命令兼容回退。

## 核心能力

`mavros_xyz_position_node` 按显式授权逐级创建控制资源：

- 默认只订阅 MAVROS、传感器和 LCP 话题，执行预检和状态输出，不创建设定点发布器。
- 满足位置控制确认组后，才创建 `/mavros/setpoint_raw/local` 发布器。
- 满足模式确认后，才创建 `/mavros/set_mode` 客户端；满足飞行确认后，才创建
  `/mavros/cmd/arming` 客户端。
- 所有模式和解锁结果都必须由后续 `/mavros/state` heartbeat 确认；服务响应本身不代表飞控
  状态已经改变。
- 通过固定远端、IP/端口白名单的 UDP 发送 Plan1 JSON，并拒绝错误方向、重复键、未知字段、
  非 UTF-8 或无效数值的入站报文。
- 按阶段执行“B 点直达、视觉卡尔曼预测投放、自主返航、下降、普通解锁、`MANUAL`”任务流程。

## 内部架构

```text
GCS UDP ──> GroundStationLink ──> Navigation ──> Offboard ──> MAVROS / PX4
                  ▲                    │
                  │                    └────────> PwmGripper
                  │
/lcp/debug ───────┼──> xyzstatus
Initialization ────┴──> 健康状态与相对 Z
LcpVisionBridge ─────────> LCP NWU 到 ENU 外部视觉位姿
```

`GroundStationLink` 负责 UDP、JSON 校验、离散事件 ACK 队列和 `xyzstatus` 编码；
`Navigation` 是不依赖 ROS 的纯值类型任务状态机；`PwmGripper` 是非阻塞 PWM sysfs
适配器；`ApplicationNode` 在 20 Hz 单线程定时器中连接遥测、飞控服务、导航和通信。
控制超时使用单调时钟，不受 ROS 时间跳变影响。

## GCS-UAV V0.8 通信

每个 UDP 数据报恰好包含一个 UTF-8 JSON 对象：

```json
{"header":"<name>","data":{}}
```

UAV 读取 `udp.bind_*`、`udp.remote_*`、`udp.whitelist_*` 参数。入站数据报的源 IP **和**
源端口必须同时匹配白名单；出站数据报只发送至固定远端。

| 方向 | 消息 | 含义 |
| --- | --- | --- |
| GCS -> UAV | `run_plan1` | 在 `ok_wait` 后启动 OFFBOARD、ARM 和 1.5 m 爬升。 |
| GCS -> UAV | `car_status` | 在 ACK `ok_b` 后持续发送目标相对距离与相对方位。 |
| GCS -> UAV | `ack` | 确认当前最早的未确认 `ok_*` 事件。 |
| UAV -> GCS | `ok_wait`、`ok_b`、`ok_throw`、`ok_return`、`ok_downing`、`ok_down` | 有序、需要 ACK 的离散阶段事件。 |
| UAV -> GCS | `xyzstatus` | 每个新 `/lcp/debug` 样本的 LCP 数据和相对 Z 元数据；不需要 ACK。 |

离散 `ok_*` 事件按顺序入队，首个事件立即发送，并以
`udp.event_retry_period_s`（默认 `0.5 s`）重发，直至收到 `{"header":"ack","data":{}}`。
`xyzstatus` 不进入该队列，不能 ACK。

到达高度并连续稳定 3 s 后，UAV 从 ARM 锁存的 Init 与初始偏航直达 B 点，保持高度和初始偏航。
到 B 点并实测落入容差后仅发送一次 `ok_b`。GCS ACK 后开始特定图形识别，并持续发送
`car_status`。UAV 用收包时间和实测 yaw 进行 ENU 转换，以二维匀速卡尔曼滤波预测首次进入
0.20 m 投放距离的时刻。预测窗口大于 0.5 s 时只整形平移到最近直角相对方位，不转动机头；最后
0.5 s 直接拦截预测目标。旧 `ok_height`、`go_ahead_ok`、`match_car_ok` 和 `b_ok` 一律拒绝并
记录 `legacy_header_rejected`。

## 任务状态机

启用完整控制并通过预检后，正常任务流程为：

```text
预检与 LCP 初始化
  -> 锁存 Init XYZ、发送 ok_wait
  -> 等待 run_plan1
  -> 设定点预热 -> OFFBOARD -> 普通 ARM
  -> 相对 Init 爬升 1.5 m、连续稳定 3 s
  -> B 点直达 -> ok_b -> 等待连续 car_status
  -> 直角方位整形 -> 最终预测拦截 -> PWM 投放 -> ok_throw
  -> 自主返航 Init XY、世界偏航 0 -> ok_return
  -> 开始下降 -> ok_downing
  -> 着陆 -> 普通 Disarm -> MANUAL -> ok_down
```

`Navigation` 对 XY 和 Z 使用五次多项式轨迹，并通过延长轨迹时间限制速度和加速度。LCP 在
高度稳定、B 点、拦截、投放或返航阶段失效时，任务会冻结可靠位置；超过保持超时、飞控模式丢失、
飞行健康失败或最大飞行时间后，进入安全降落路径。

`ok_down` 的语义是已着陆、PX4 已普通解锁（Disarm）且已确认 `MANUAL`；不会关闭 Raspberry Pi
或飞控电源。

## LCP、外部视觉与 Z

`LcpVisionBridge` 将 `lcp_nwu` 里程计转换为 ROS ENU 并发布到
`/mavros/vision_pose/pose_cov`：

```text
lcp_nwu:  x=北，y=西/左，yaw=0 指北
       -> x_enu=-y_nwu，y_enu=x_nwu，yaw_enu=yaw_nwu+pi/2
lcp_enu:  x=东，y=北
```

应用还订阅 `/lcp/debug`（`lslidar_msgs/LcpDebug`），每个新样本均发送一条 `xyzstatus`。发送
`ok_wait` 时锁存 MAVROS 本地 Z 为 Init，同时记录有效向下 Range 基准。默认采用新鲜本地位姿的
`local_z - init_local_z`；仅当其与新鲜、有效 Range 的相对值差不超过
`z.range_cross_check_max_delta_m`（默认 `0.30 m`）时，Z 才有效。设置
`z.prefer_range:true` 可选择经校准的 Range 相对 Z。

无有效 Z 时，LCP 其余字段仍照常发送，且 Z 元数据固定为：

```json
"position_z_m":null,"z_source":"none","z_source_stamp":null,"z_quality":null,"z_valid":false
```

## PWM 夹爪

`PwmGripper` 使用 `/sys/class/pwm/pwmchip*`，以“延时 -> 保持释放占空比 -> 恢复空闲占空比”
的非阻塞状态机完成一次投放。`gripper_pwm.enabled:false` 是默认值，适用于普通软件构建和
SITL；此时适配器执行定时空操作，不访问硬件。

在完成无桨台架标定前，**不得**设置 `gripper_pwm.enabled:true`。启用硬件输出还要求：

- 正确的 `chip_path`、`channel`、周期、空闲占空比和释放占空比；
- `gripper_pwm.pinmux_path` 可读且包含 `gripper_pwm.pinmux_expected`；
- PWM 通道可导出，且 `period`、`duty_cycle`、`enable` 均可写。

任一准备或写入失败都不会发送 `ok_throw`，状态机会记录失败并返航；本流程不重试投放。自动测试
仅使用 sysfs 仿真目录，不驱动真实 PWM 引脚。

## 预检与显式确认

预检会检查 MAVROS heartbeat、模式、解锁和落地状态的新鲜度；电池、local pose、local velocity、
四元数、估计器状态、向下测距、光流和 LCP 健康状态。默认还要求 local Z 与向下测距差值不超过
`0.30 m`。程序不会伪造 PX4 内部融合证据；实际带桨飞行仍需独立 PX4 shell/ULog 核验和物理安全
措施。

创建位置设定点发布器前，必须提供：

```text
--enable-position-setpoints
--ack-native-xyz-position-control
--ack-setpoint-streaming-risk
--confirm-setpoint-mav-frame-local-ned
--confirm-range-source
--confirm-optical-flow-source
```

请求 OFFBOARD 前，还必须提供：

```text
--request-offboard-mode
--ack-disarmed-mode-switch
```

执行普通 ARM 和受限飞行前，还必须提供：

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

若任一确认组不完整，参数解析会报错，相关控制资源不会被创建。通过单元测试不构成飞行授权。

## 构建与测试

`lslidar_msgs` 是必需依赖。配置、构建和测试前先加载 ROS 及 LSLIDAR overlay：

```bash
cd /home/pi/px4-test-tools
source /opt/ros/jazzy/setup.bash
source /home/pi/LSLIDARN10P/install/setup.bash
colcon build --symlink-install --packages-select mavros_xyz_position_offboard --parallel-workers 1
source install/setup.bash
colcon test --packages-select mavros_xyz_position_offboard --event-handlers console_direct+
colcon test-result --all --verbose
```

测试覆盖 Plan1 入站校验、白名单拒绝、ACK 重传队列、有效/无效 Z 的 `xyzstatus` 编码、完整任务
顺序、PWM sysfs 仿真和 UDP 回环。真实 PWM 引脚和实际飞行不在自动测试范围内。

## 配置与运行

使用 ROS 参数文件加载 UDP、跟车、Z 与 PWM 默认值：

```text
/home/pi/px4-test-tools/mavros_xyz_position_offboard/config/udp_ground_station.yaml
```

只读监视示例：

```bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node \
  --confirmed-fcu-url udp://127.0.0.1:14540 \
  --range-topic /mavros/px4flow/ground_distance --range-source-label downward \
  --optical-flow-topic /mavros/px4flow/raw/optical_flow_rad --optical-flow-source-label flow \
  --ros-args --params-file \
  /home/pi/px4-test-tools/install/mavros_xyz_position_offboard/share/mavros_xyz_position_offboard/config/udp_ground_station.yaml
```

执行顺序应为：无桨台架与单独 PWM 标定、PX4 SITL、受控实机飞行。每一阶段都应记录 UDP 数据报及
`/lcp/debug`、MAVROS 时间戳，并重新核验飞控连接、传感器方向、螺旋桨/区域安全、PX4 XY 融合证据
和独立急停。

## 日志与目录

节点默认在 `artifacts/` 创建 `mavros-xyz-flight-<UTC>-<pid>.log`。指定 `--output jsonl`
后生成同名 `.jsonl`，记录任务阶段、拒绝原因、飞控状态、遥测、LCP、服务请求和审计配置。

```text
px4-test-tools/
├── mavros_xyz_position_offboard/       ROS 2 C++17 UAV 节点、库、测试和包级 README
│   ├── src/communication/              Plan1 UDP 协议、ACK 队列与 xyzstatus 编码
│   ├── src/navigation/                 五次轨迹和任务状态机
│   ├── src/gripper/                    非阻塞 Linux PWM sysfs 夹爪适配器
│   ├── src/initialization/             遥测订阅、预检、RangeGuard 与 LCP 门禁
│   ├── src/bridge/                     LCP NWU 到 ENU 外部视觉桥
│   └── test/                           C++ 单元与 UDP 回环测试
├── lcp_yaw_vision_bridge.py            独立 Python LCP 到 MAVROS 外部视觉桥
├── read_px4_params.py                  PX4 参数读取辅助工具
├── command.txt                         经过审计的命令模板
└── artifacts/                          本地运行产物，已忽略
```

独立 Python 桥接脚本和 C++ 节点内置桥接不能同时运行，否则会向同一外部视觉话题重复发布。
