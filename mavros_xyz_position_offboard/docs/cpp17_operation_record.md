# C++17 ROS 2 包操作记录

## 目录与职责

| 路径 | 职责 |
| --- | --- |
| `application/` | 唯一 `ApplicationNode`，组装各层并执行固定 20 Hz 单线程循环。 |
| `initialization/` | ROS 遥测、预检、飞行健康快照和 LCP 初始化服务。 |
| `communication/` | JSON V1 DTO/codec、UDP socket、白名单、去重和周期状态。 |
| `navigation/` | 无 ROS 的任务状态机和 `TrajectoryPlanner`。 |
| `offboard/` | MAVROS PositionTarget、SetMode 和 CommandBool 执行适配器。 |
| `bridge/` | LCP NWU 到 MAVROS ENU vision pose。 |
| `common/` | CLI、安全配置、值类型和 artifact 日志。 |
| `config/udp_ground_station.yaml` | 固定端点、白名单、状态周期和目标安全包络。 |

唯一节点名保持 `mavros_native_xyz_position`，唯一执行器保持
`SingleThreadedExecutor`。模块间不创建内部 ROS 话题。

## 构建、测试与安装入口

```bash
cd /home/pi/px4-test-tools
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mavros_xyz_position_offboard
colcon test --packages-select mavros_xyz_position_offboard
colcon test-result --all --verbose
source install/setup.bash
ros2 run mavros_xyz_position_offboard mavros_xyz_position_node --help
```

本包没有 `lslidar_msgs` 依赖，不需要 LSLIDAR overlay。测试覆盖 LCP 初始化基线、NWU/ENU
转换、五次轨迹约束、全部 V1 输入消息、白名单/版本/去重/乱序/畸形包、三份配置一致性、
OFFBOARD/ARM 确认、ACK hold/恢复、PositionTarget mask、OFFBOARD 结构边界和 CLI 门禁。

## 控制循环

每个 wall timer tick 的顺序不可调整：

1. `Initialization::poll_lcp_start`、`Offboard::poll`、`GroundStationLink::poll`；
2. `Initialization::health_snapshot` 复制本周期不可变遥测与健康结论；
3. `Navigation::update` 生成阶段、设定点、模式/ARM 意图和业务输出；
4. `Offboard::apply` 发布设定点并按限频规则提交异步服务请求；
5. `GroundStationLink` 发送业务消息以及周期 XYZ/电池状态；
6. 写入 `px4.mavros_native_xyz.v1` 状态日志。

日志新增 `navigation_batch.waypoint_index`、`ack_age_s`、
`communication_rejection` 和 `navigation_rejections`。服务 response 不代表飞控已经执行；
Navigation 只认下一条 MAVROS state 中的实际 mode/armed 值。

## UDP 现场检查

安装后的参数文件位于：

```text
share/mavros_xyz_position_offboard/config/udp_ground_station.yaml
```

修改参数后必须重启节点。确认地面站数据报源地址和源端口都匹配白名单；`seq` 只需唯一，
无需按到达顺序递增。批次发送顺序建议为三份 `navigation_and_point`、三份
`navigation_nfz`、三份 `navigation_plan`、一个 `navigation_fly_plan_send_ok`，飞行期间
至少 2 Hz 发送 `ACK`。

节点不会对输入数据报返回逐包 ACK。用抓包检查时，只应看到阶段业务消息和按
`udp.status_period_s` 发送的 `xyz_state`/`battery_state`。

## 控制授权和现场边界

CLI 三层确认保持不变：

1. PositionTarget publisher 的原生 XYZ、streaming 风险、LOCAL_NED 和传感器来源确认；
2. SetMode client 的 OFFBOARD 和 disarmed mode switch 确认；
3. CommandBool client 的有界飞行、普通 ARM、桨叶/场地/急停/电池和 PX4 XY 融合证据确认。

缺少任何同层配套参数会在节点创建前拒绝。连接实机前仍需重新核对 FCU URL、Range
方向、光流来源、LCP 状态、地面站固定端点、空间隔离及 PX4 原生融合证据。本文记录的
构建/单元测试不等同于带桨飞行验收。

## 故障行为

- ACK 年龄超过 2 秒：`link_hold`，冻结实测 XY/命令 Z；ACK 恢复后连续重规划。
- LCP 失效：`lcp_hold`；恢复后继续原目标，超时请求安全降落。
- 飞行健康门禁持续失败超过 grace、OFFBOARD 丢失或最大飞行时间超限：安全降落；模式
  丢失不抢回 OFFBOARD。
- 正常任务完成：等待 `ok_fly_plan_succeed`，随后回到初始化 Z、Disarm，确认后请求
  `MANUAL`。
