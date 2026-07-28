# ROS 2 MAVROS 原生 XYZ 任务节点

本包面向 PX4 1.17、ROS 2 Jazzy 和 MAVROS 2.14，以单进程、单 ROS 2 节点、20 Hz
单线程循环执行 OFFBOARD XYZ+yaw 任务。它不修改 PX4/MAVROS 参数，也没有
force-arm/force-disarm 路径。

详细构建和现场操作见 [C++17 操作记录](docs/cpp17_operation_record.md)。

## 架构和循环顺序

唯一 ROS 节点是 `application::ApplicationNode`，节点名仍为
`mavros_native_xyz_position`。内部依赖方向固定如下：

```text
GroundStationLink --解析事件--> Navigation --OffboardCommand--> Offboard --> PX4
Initialization --------健康快照----^
LcpVisionBridge: LCP NWU --> MAVROS ENU
```

- `Initialization`：MAVROS/LCP 遥测缓存、预检、飞行健康、LCP 初始化服务；不决定任务阶段。
- `GroundStationLink`：非阻塞 UDP、白名单、严格 JSON V1 编解码、去重和状态发送。
- `Navigation`：纯 C++ 任务状态机、三包批次、航点队列、hold/恢复和安全决策；不引用 ROS、MAVROS、socket 或 JSONCPP。
- `TrajectoryPlanner`：连续、速度/加速度有界的 XY/Z 五次轨迹。
- `Offboard`：设定点映射/发布及 SetMode、CommandBool 异步请求；不引用通信或任务阶段。
- `LcpVisionBridge`：把 `lcp_nwu` XY+yaw 转为 ROS ENU vision pose。

定时器每周期严格执行：轮询服务结果和 UDP → 获取不可变健康快照 → Navigation →
Offboard → GroundStationLink 发送 → 统一日志。

## 飞行时序

任务阶段为：

```text
waiting_preflight -> waiting_start -> setpoint_warmup
-> offboard_request_pending -> arming_request_pending -> climb -> stabilize
-> waiting_navigation_config -> run_fly_plan
-> awaiting_fly_plan_succeed -> landing -> disarming
-> manual_request_pending -> manual
```

运行中还可能进入 `link_hold` 或 `lcp_hold`。

1. 完整预检和 LCP 初始化就绪后发送 `ok_preflight`，等待 `start`。
2. `height_start` 必须在 0.2～1.0 m；它是相对初始化 local ENU Z 的高度。
3. 先持续预热原点设定点，再请求 OFFBOARD 和 ARM。服务 response 只记日志；仅在
   `/mavros/state` 实际确认 `OFFBOARD` 且 `armed=true` 后发送 `ok_flight` 并爬升。
4. 位置稳定 `hold_seconds` 后发送 `wait_plan` 并收集导航批次。
5. `plan_mode=1` 按航点数组顺序执行；若最后航点不是 `navigation_and_point.point`，自动追加终点。
   航点切换要求轨迹结束且实测 XY 欧氏距离不超过 0.2 m；Z 发送但不参与协议完成判定。
6. 最终目标到达后保持位置并发送 `ok_fly_plan`。收到 `ok_fly_plan_succeed` 才下降到
   初始化 Z、Disarm，最后请求并确认 `MANUAL`。

`plan_mode=0` 本版本明确拒绝并记录 `autonomous_planning_not_supported`。飞行中不接受
替换/追加任务。NFZ 本版本仅做数组、数量、类型、有限性和三份一致性校验。

ACK 是 GCS→UAV 心跳。`run_fly_plan` 连续 2 秒无有效 ACK 时，在最新可靠实测 XY 和
当前命令 Z 进入 `link_hold`；恢复 ACK 后从冻结状态平滑重规划到原活动航点。LCP
失效进入 `lcp_hold`，恢复后同样连续重规划，超时则安全降落。模式丢失不会抢回
OFFBOARD，而是请求正常 `AUTO.LAND`。

## JSON UDP V1

参数文件为 `config/udp_ground_station.yaml`。默认仅接收
`192.168.10.59:5005`，发送也固定到该端点。所有数据报必须是 UTF-8 JSON 对象并包含：

```json
{"version":1,"type":"ACK","seq":42}
```

`version` 必须为整数 1，`type` 必须为字符串，`seq` 必须为无符号整数。`seq` 在进程
生命周期内用于去重，但允许 UDP 乱序。重复字段、额外字段、错误类型、非有限数值、
畸形数组、点数不符和非白名单来源均拒绝。接收器不发送旧式逐包 ACK。

GCS→UAV 消息：

```json
{"version":1,"type":"start","seq":1,"height_start":0.8}
{"version":1,"type":"navigation_and_point","seq":2,"plan_mode":1,"point":{"x":2.0,"y":3.0,"z":0.8}}
{"version":1,"type":"navigation_nfz","seq":3,"nfz_point_count":1,"nfz_points":[{"x":1.0,"y":1.0,"z":0.0}]}
{"version":1,"type":"navigation_plan","seq":4,"waypoint_count":2,"waypoints":[{"x":1.0,"y":2.0,"z":0.8},{"x":2.0,"y":2.0,"z":0.8}]}
{"version":1,"type":"navigation_fly_plan_send_ok","seq":5}
{"version":1,"type":"ACK","seq":6}
{"version":1,"type":"ok_fly_plan_succeed","seq":7}
```

在 `waiting_navigation_config` 中，前三种导航载荷必须各收到恰好三份且三份业务载荷
完全一致，随后收到一个 `navigation_fly_plan_send_ok`。缺失、不一致或数量超限会清空
整批并保持悬停，不发送 `ok_receive`。

UAV→GCS 消息为 `ok_preflight`、`ok_flight`、`wait_plan`、`ok_receive`、
`ok_fly_plan`、`xyz_state` 和 `battery_state`。所有输出也带 V1 包络和 UAV 自增 `seq`。
`xyz_state` 是 MAVROS local ENU 绝对 XYZ；`battery_state` 包含 `present`、`voltage` 和
`remaining`。两类状态默认每 0.5 秒发送一次，可用 `udp.status_period_s` 修改。

导航目标继续受 YAML 安全包络限制：XY 相对初始化 local ENU 原点，Z 为绝对 local ENU。
NFZ 点不作为控制目标，因此不套用飞行目标包络。

## 构建与测试

```bash
cd /home/pi/px4-test-tools
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mavros_xyz_position_offboard
colcon test --packages-select mavros_xyz_position_offboard
colcon test-result --all --verbose
source install/setup.bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node --help
```

本包不再依赖 `lslidar_msgs`，构建和测试无需 source LSLIDAR overlay。

## 运行与安全授权

只读监视示例：

```bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node \
  --confirmed-fcu-url udp://127.0.0.1:14540 \
  --range-topic /mavros/px4flow/ground_distance --range-source-label downward \
  --optical-flow-topic /mavros/px4flow/raw/optical_flow_rad --optical-flow-source-label flow \
  --ros-args --params-file \
  /home/pi/px4-test-tools/install/mavros_xyz_position_offboard/share/mavros_xyz_position_offboard/config/udp_ground_station.yaml
```

默认不创建设定点 publisher、SetMode client 或 CommandBool client。控制资源仍由原有 CLI
三层显式确认逐级开启：PositionTarget 确认 → OFFBOARD 模式确认 → 有界飞行与普通 ARM
确认。缺少任一配套确认会在创建 ROS 资源前退出。所有传感器、电池、估计器、落地状态、
LCP 新鲜度和本地位姿门禁继续生效；节点不把 MAVROS 可见字段包装成不存在的 PX4 原生
融合证据。
