# LCP 启动 API 与 Offboard 接入

本文定义 `LSLIDARN10P` 驱动侧 LCP 的启动接口和 Offboard 使用规则。LCP 使用完整一圈 N10P 扫描建系，并在人工声明机头正北时输出独立的 `lcp_nwu` 局部位姿；LCP 本身**不会**直接向 PX4/MAVROS 注入外部里程计。

## 前置条件

先启动 MAVROS，再启动 N10P 驱动。驱动在建系时订阅：

- `/mavros/state`
- `/mavros/extended_state`
- `/mavros/imu/data`

启动建系服务只有在以下条件同时满足时才会接受请求：MAVROS 已连接、飞控未解锁、飞行器状态为 `ON_GROUND`，且飞控状态消息没有超时。

## 启动服务

| 名称 | 类型 | 作用 |
| --- | --- | --- |
| `/lcp/start_initialization` | `std_srvs/srv/Trigger` | 清空旧地图并开始一次新的 LCP 建系 |

调用示例：

```bash
source /opt/ros/jazzy/setup.bash
source ~/LSLIDARN10P/install/setup.bash
ros2 service call /lcp/start_initialization std_srvs/srv/Trigger '{}'
```

只有 `success: true` 且返回 `LCP initialization started` 才表示请求已受理。若服务失败，Offboard 必须保持地面等待，不得请求 Offboard 模式或解锁。

## 输出话题

| 名称 | 类型 | 发布条件 | 含义 |
| --- | --- | --- | --- |
| `/lcp/status` | `std_msgs/msg/UInt8` | 每圈完整扫描 | LCP 状态值 |
| `/lcp/odometry` | `nav_msgs/msg/Odometry` | 仅 `STATUS=2` | `lcp_nwu` 中的 XY 与 yaw 四元数 |
| `/lcp/yaw` | `std_msgs/msg/Float32` | 仅 `STATUS=2` | 相对人工声明北方的 yaw，单位 rad |
| `/lcp/debug` | `lslidar_msgs/msg/LcpDebug` | 仅 `STATUS=2` | 机体前后左右到已锁定矩形边界的绝对距离，以及锁定地图 XY 尺寸 |

默认 `lcp_initial_heading_is_north=true`。在调用启动服务前，必须让**雷达 +X 与机体正前方对齐**，并把机体正前方实际摆向北方。LCP 在锁图的首帧将该方向声明为北：`header.frame_id=lcp_nwu`，`+X=北`、`+Y=西/左`、`+Z=上`，建系点为 `(0,0)`，`/lcp/yaw=0 rad`。此北向来自人工放置，不是磁力计或卫星测得的方位；移动设备、雷达安装偏角或摆放误差会等量带入所有后续 XY/yaw。

`/lcp/debug` 的 `front/rear/left/right_distance_m` 以当前雷达/机体方向为准（+X/+Y/-X/-Y），并通过与已锁定地图边界求交得到正距离；它不把建系原点的 `(0,0)` 当作墙距。`map_size_x_m/map_size_y_m` 是内部锁定矩形的边长，不因公开的北向坐标旋转而重新命名。

## STATUS 状态机

| 值 | 名称 | Offboard 行为 |
| --- | --- | --- |
| `0` | 初始化前 | 禁止起飞；先调用启动服务。 |
| `1` | 初始化中 | 禁止起飞；继续等待稳定矩形地图。 |
| `2` | 初始化后/健康 | 允许进入 Offboard warmup 的必要条件；要求 status 和 odometry 均持续新鲜。 |
| `3` | 不健康或不水平 | 禁止新的移动指令；驱动停止发布新 XY/yaw。 |

`STATUS=3` 可由倾角超过驱动阈值（默认 roll 或 pitch 超过 `15°`）、MAVROS 姿态或飞控状态超时、扫描超时、矩形拟合失败或地图匹配失败触发。它不会在空中重建或切换坐标轴；重新获得健康匹配后可恢复为 `STATUS=2`。

## Offboard 最小接入顺序

1. 完成现有 MAVROS、电池、测距、光流与地面预检。
2. 调用 `/lcp/start_initialization`，必须确认服务成功。
3. 等待 `/lcp/status=2`，并确认 `/lcp/odometry` 是新鲜数据；建议要求连续至少 3 个健康扫描。
4. 满足上述条件后，才开始现有 setpoint warmup、申请 `OFFBOARD` 和解锁。
5. 飞行中若收到 `STATUS=3`，或 status/odometry 超时：冻结航点推进，持续发布当前 MAVROS 本地位置的 XY 固定点与当前高度目标。
6. 连续恢复到新鲜 `STATUS=2` 后，才恢复原飞行阶段；若不健康超过 Offboard 配置的等待上限，应转入安全降落。

Offboard 不应把 `/lcp/odometry` 的数值直接写入 `/mavros/setpoint_raw/local`。`lcp_nwu` 使用北/西/上，而 MAVROS ROS 话题使用东/北/上；二者必须先做完整 XY+yaw 变换。当前阶段，LCP 用作建系和健康门禁；后续若要替换磁罗盘，需要单独实现 `LCP Odometry -> MAVROS ODOMETRY -> PX4 EKF2` 融合链路。

## LCP 外部 yaw 台架试验（2026-07-25）

### 当前结论

LCP 的 yaw 和四向墙距已在网罩内完成静态、手动转向验证，但**尚未获准替代
PX4 磁力计或用于飞行**。可用于 PX4 EKF2 的候选量只有 `/lcp/odometry` 的
`pose.pose.position.x/y` 和 yaw；`/lcp/debug` 的四向墙距与地图尺寸只用于质量
检查和机外避障逻辑，不能直接作为 EKF 状态量。

手动将机头旋转约 180° 时，观察到：

| 字段 | 旋转前 | 旋转后 | 预期 |
| --- | ---: | ---: | --- |
| `yaw_rad` | `1.7104` | `-1.4312` | 差值约 `-π` |
| front / rear (m) | `2.2528 / 1.9710` | `2.0143 / 2.2095` | 前后近似互换 |
| left / right (m) | `2.9537 / 2.2165` | `2.2604 / 2.9098` | 左右近似互换 |
| map size X / Y (m) | `5.1199 / 4.1827` | `5.1199 / 4.1827` | 锁定地图不变 |

数厘米的差异与两次原地转向时 LCP 的位置估计变化一致。该试验只说明局部几何和
航向方向正确；没有给出飞行振动、快速转向、失锁恢复或长期漂移的精度结论。

### 已验证的传输链路与当前限制

LCP 到 MAVROS 的桥接已内置于 C++ ROS 2 包
`mavros_xyz_position_offboard`。启动 `mavros_xyz_position_node` 时会自动创建；默认只
发布外部视觉消息，不创建设定点 publisher、不会请求 Offboard 或解锁。旧
`~/px4-test-tools/lcp_yaw_vision_bridge.py` 保留为迁移前参考，不能与 C++ 节点同时运行。

```text
/lcp/odometry (STATUS=2)
  -> mavros_xyz_position_node 内置 LcpVisionBridge
  -> /mavros/vision_pose/pose_cov
  -> MAVLink VISION_POSITION_ESTIMATE
  -> PX4 vehicle_visual_odometry
```

该桥仅接受 `header.frame_id=lcp_nwu` 和新鲜 `STATUS=2`，固定做完整
`NWU -> ENU` 变换：`east=-west`、`north=north`、`yaw_enu=pi/2+yaw_nwu`。XY/yaw
标准差默认 `0.20`，z/roll/pitch 方差 `10000`；它不读取 PX4 当前 yaw，也不会设置
PX4 参数。

台架中 PX4 已接收到 `vehicle_visual_odometry`，其 yaw 方差为 `0.0400`，采样到
飞控接收延迟约 `44 ms`。但当时的 PX4 基线参数为：

| 参数 | 基线值 | 含义 |
| --- | ---: | --- |
| `EKF2_EV_CTRL` | `0` | 未启用外部视觉融合 |
| `EKF2_EV_QMIN` | `50` | 外部视觉质量门限 |
| `EKF2_MAG_TYPE` | `0` | 磁力计自动融合 |
| `SYS_HAS_MAG` | `1` | 磁力计存在且启用 |

MAVROS 的 `VISION_POSITION_ESTIMATE` 路径使该次消息在 PX4 中显示
`quality=0`。在临时把 `EKF2_EV_CTRL` 设为 `8`（仅外部 yaw）后，
`cs_ev_yaw` 仍为 `false`，原因是 `quality=0 < EKF2_EV_QMIN=50`。该临时设置已
恢复为 `EKF2_EV_CTRL=0`；没有禁用磁力计，也没有飞行。

### NSH 与 MAVROS 互斥规则

`~/px4-emergency/px4_nsh_snapshot.py` 直接打开 FCU 的 MAVLink 串口。不得在
MAVROS 已连接同一飞控时并发执行它，否则两个进程会竞争 MAVLink 字节流，可能使
MAVROS 失去心跳和所有 `/mavros/*` 发布者。

必须遵守以下顺序：

1. 要读取 NSH/uORB/参数：先停止 MAVROS，执行 `px4_nsh_snapshot.py`，结束后再启动 MAVROS。
2. MAVROS 运行期间：不要执行该脚本；参数写入使用 MAVROS 的参数服务，且只在地面、未解锁时进行。
3. 每次重启 MAVROS 后，先确认 `/mavros/state` 为 `connected: true`、
   `/mavros/extended_state` 为 `ON_GROUND`，以及 `/lcp/status=2`，才允许后续 LCP 测试。

### 下一步的最低安全门槛

若继续验证外部 yaw，必须保持地面、螺旋桨不转，并在每轮试验后恢复参数基线。
首先需要解决 MAVLink 外部视觉 `quality=0` 与 `EKF2_EV_QMIN=50` 的匹配问题；
之后才能确认 `cs_ev_yaw=true`。在 10 Hz 的 LCP 更新率、失锁处理、安装 yaw 偏置、
协方差和延迟完成量化验证前，不得关闭磁力计或尝试起飞。

## 运行时核验

```bash
ros2 service type /lcp/start_initialization
ros2 topic info /lcp/status
ros2 topic echo /lcp/status
ros2 topic echo /lcp/odometry
```

正常建系应观察到状态顺序 `0 -> 1 -> 2`。在 `STATUS=2` 前或 `STATUS=3` 时，不应收到新的 `/lcp/odometry` 和 `/lcp/yaw`。
