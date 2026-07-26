# Kilo Memory

Root: /home/pi/.local/share/kilo/memory/px4-test-tools-bd496d0360db
Enabled: yes
Auto-save: on
Startup context: on
Stored index tokens: 592
Startup context tokens for this session: 0
Last auto-save model usage: 2873 tokens

## project.md
# Project Memory

## Facts
- px4_ekf_optical_flow_fusion_params :: PX4 1.17 EKF2 光流/测距融合关键参数(MTF02P 接 TELEM2 作为 MAVLink 设备): 必须启用(融合开关): - EKF2_OF_CTRL=1: 光流融合启用 - EKF2_RNG_CTRL=2: 测距条件融合(2=conditional, 受 EKF2_RNG_A_HMAX/VMAX 限制)。也可设 1=总是融合 - EKF2_HGT_REF=2: 高度参考用测距传感器(2=range)。地面静止时 local_z 应接近 range,差...
- electroniccompetion :: ElectronicCompetion
- mavros_xyz_offboard_design :: mavros_xyz_position_offboard 节点设计要点: 核心数据流: - 读: /mavros/local_position/pose(ENU), velocity_local, estimator_status, px4flow/raw/optical_flow_rad, px4flow/ground_distance - 写: /mavros/setpoint_raw/local (PositionTarget, ENU, MAVROS 转 NED...
- px4_test_tools_workspace :: 工作区: /home/pi/px4-test-tools/ 硬件: PX4 FMUv6C.x / Pi 5 / MTF02P(光流+测距一体模块,接飞控 TELEM2,作为 MAVLink 设备向飞控发数据) 固件: PX4 1.17.0, ROS 2 Jazzy, MAVROS 2.14.0 ROS 2 包(ament_python, colcon build): - mavros_xyz_position_offboard: XYZ 位置 OFFBOARD 控制节点...
- px4_emergency_tools :: PX4 调试/应急工具目录: ~/px4-emergency/ 关键工具(通过 MAVLink 直连飞控,不走 MAVROS,只读安全): - px4_nsh_snapshot.py: 执行 NSH 命令。用法: python3 px4_nsh_snapshot.py --command "命令" [--command "..."] [--wait 秒数]。支持 listener/uorb top/param set/param show/reboot 等。设备默认 /...

## Decisions

## Constraints

## Open Questions

## environment.md
# Environment Memory

## Commands

## Paths

## Tooling

## corrections.md
# Corrective Memory

## Corrections

## index.kmem
```kilo-memory-v1 context_not_instruction
scope: project
root: px4-test-tools-bd496d0360db
limits: 8192/5/480

record id=latest_session.ses_084c789e5ffe19qdYlxvTv9C7t type=latest_session_digest source=ses_084c789e5ffe19qdYlxvTv9C7t.md updated=2026-07-19T17:58:17.234Z
text: session=ses_084c789e5ffe19qdYlxvTv9C7t topic="飞控重启与EKF2验证" 2026-07-19T17:58:17.234Z :: 用户发送命令让飞控reboot，飞控已成功重启并重连。下一步需等待约30秒让EKF2完全收敛，然后用只读模式验证 local_z 是否接近 range（差值应 <0.3m），确认高度数据正常后再决定是否起飞。
record id=project.md_Facts_px4_ekf_optical_flow_fusion_params type=project_fact source=project.md updated=2026-07-19T18:01:22.340Z
text: px4_ekf_optical_flow_fusion_params :: PX4 1.17 EKF2 光流/测距融合关键参数(MTF02P 接 TELEM2 作为 MAVLink 设备): 必须启用(融合开关): - EKF2_OF_CTRL=1: 光流融合启用 - EKF2_RNG_CTRL=2: 测距条件融合(2=conditional, 受 EKF2_RNG_A_HMAX/VMAX 限制)。也可设 1=总是融合 - EKF2_HGT_REF=2: 高度参考用测距传感器(2=range)。地面静止时 local_z 应接近 range,差...
record id=project.md_Facts_electroniccompetion type=project_fact source=project.md updated=2026-07-19T18:01:22.339Z
text: electroniccompetion :: ElectronicCompetion
record id=project.md_Facts_mavros_xyz_offboard_design type=project_fact source=project.md updated=2026-07-19T18:01:22.338Z
text: mavros_xyz_offboard_design :: mavros_xyz_position_offboard 节点设计要点: 核心数据流: - 读: /mavros/local_position/pose(ENU), velocity_local, estimator_status, px4flow/raw/optical_flow_rad, px4flow/ground_distance - 写: /mavros/setpoint_raw/local (PositionTarget, ENU, MAVROS 转 NED...
record id=project.md_Facts_px4_test_tools_workspace type=project_fact source=project.md updated=2026-07-19T18:01:22.337Z
text: px4_test_tools_workspace :: 工作区: /home/pi/px4-test-tools/ 硬件: PX4 FMUv6C.x / Pi 5 / MTF02P(光流+测距一体模块,接飞控 TELEM2,作为 MAVLink 设备向飞控发数据) 固件: PX4 1.17.0, ROS 2 Jazzy, MAVROS 2.14.0 ROS 2 包(ament_python, colcon build): - mavros_xyz_position_offboard: XYZ 位置 OFFBOARD 控制节点...
record id=project.md_Facts_px4_emergency_tools type=project_fact source=project.md updated=2026-07-19T18:01:22.336Z
text: px4_emergency_tools :: PX4 调试/应急工具目录: ~/px4-emergency/ 关键工具(通过 MAVLink 直连飞控,不走 MAVROS,只读安全): - px4_nsh_snapshot.py: 执行 NSH 命令。用法: python3 px4_nsh_snapshot.py --command "命令" [--command "..."] [--wait 秒数]。支持 listener/uorb top/param set/param show/reboot 等。设备默认 /...
record id=topic.map type=topic_hint source=inventory updated=2026-07-19T18:01:22.340Z
text: topic=project sources=project.md records=5
```

## items
- id=project.md:Facts:px4_ekf_optical_flow_fusion_params type=project_fact source=project.md section=Facts key=px4_ekf_optical_flow_fusion_params topics=project terms=px4_ekf_optical_flow_fusion_params,px4,ekf,optical,flow,fusion updated=2026-07-19T18:01:22.340Z created=2026-07-19T18:01:22.340Z timeSource=source_mtime_line_offset stale=no expires=never :: PX4 1.17 EKF2 光流/测距融合关键参数(MTF02P 接 TELEM2 作为 MAVLink 设备): 必须启用(融合开关): - EKF2_OF_CTRL=1: 光流融合启用 - EKF2_RNG_CTRL=2: 测距条件融合(2=conditional, 受 EKF2_RNG_A_HMAX/VMAX 限制)。也可设 1=总是融合 - EKF2_HGT_REF=2: 高度参考用测距传感器(2=range)。地面静止时 local_z 应接近 range,差...
- id=project.md:Facts:electroniccompetion type=project_fact source=project.md section=Facts key=electroniccompetion topics=project terms=electroniccompetion,electronic,competion updated=2026-07-19T18:01:22.339Z created=2026-07-19T18:01:22.339Z timeSource=source_mtime_line_offset stale=no expires=never :: ElectronicCompetion
- id=project.md:Facts:mavros_xyz_offboard_design type=project_fact source=project.md section=Facts key=mavros_xyz_offboard_design topics=project terms=mavros_xyz_offboard_design,mavros,xyz,offboard,design,mavros_xyz_position_offboard updated=2026-07-19T18:01:22.338Z created=2026-07-19T18:01:22.338Z timeSource=source_mtime_line_offset stale=no expires=never :: mavros_xyz_position_offboard 节点设计要点: 核心数据流: - 读: /mavros/local_position/pose(ENU), velocity_local, estimator_status, px4flow/raw/optical_flow_rad, px4flow/ground_distance - 写: /mavros/setpoint_raw/local (PositionTarget, ENU, MAVROS 转 NED...
- id=project.md:Facts:px4_test_tools_workspace type=project_fact source=project.md section=Facts key=px4_test_tools_workspace topics=project terms=px4_test_tools_workspace,px4,test,tools,workspace,工作区 updated=2026-07-19T18:01:22.337Z created=2026-07-19T18:01:22.337Z timeSource=source_mtime_line_offset stale=no expires=never :: 工作区: /home/pi/px4-test-tools/ 硬件: PX4 FMUv6C.x / Pi 5 / MTF02P(光流+测距一体模块,接飞控 TELEM2,作为 MAVLink 设备向飞控发数据) 固件: PX4 1.17.0, ROS 2 Jazzy, MAVROS 2.14.0 ROS 2 包(ament_python, colcon build): - mavros_xyz_position_offboard: XYZ 位置 OFFBOARD 控制节点...
- id=project.md:Facts:px4_emergency_tools type=project_fact source=project.md section=Facts key=px4_emergency_tools topics=project terms=px4_emergency_tools,px4,emergency,tools,调试,应急工具目录 updated=2026-07-19T18:01:22.336Z created=2026-07-19T18:01:22.336Z timeSource=source_mtime_line_offset stale=no expires=never :: PX4 调试/应急工具目录: ~/px4-emergency/ 关键工具(通过 MAVLink 直连飞控,不走 MAVROS,只读安全): - px4_nsh_snapshot.py: 执行 NSH 命令。用法: python3 px4_nsh_snapshot.py --command "命令" [--command "..."] [--wait 秒数]。支持 listener/uorb top/param set/param show/reboot 等。设备默认 /...

## changes
2026-07-19T17:57:58.136Z enable project source=command
2026-07-19T17:57:58.156Z regenerate index.kmem bytes=0 [redacted]
2026-07-19T17:58:01.372Z regenerate index.kmem bytes=0 [redacted]
2026-07-19T17:58:20.480Z regenerate index.kmem bytes=626 [redacted]
2026-07-19T17:58:20.481Z session digest session=ses_084c789e5ffe19qdYlxvTv9C7t [redacted] indexTokens=118
2026-07-19T17:58:20.566Z consolidate trigger=turn-close digest=1 ops=0 [redacted] reason=in_progress
2026-07-19T17:59:25.119Z regenerate index.kmem bytes=1235 [redacted]
2026-07-19T17:59:25.131Z apply ops=1 removed=0
2026-07-19T17:59:45.109Z regenerate index.kmem bytes=1235 [redacted]
2026-07-19T17:59:45.122Z apply ops=0 removed=0
2026-07-19T17:59:54.202Z regenerate index.kmem bytes=1701 [redacted]
2026-07-19T17:59:54.204Z apply ops=1 removed=0
2026-07-19T18:00:11.754Z regenerate index.kmem bytes=2129 [redacted]
2026-07-19T18:00:11.756Z apply ops=1 removed=0
2026-07-19T18:00:37.866Z regenerate index.kmem bytes=2294 [redacted]
2026-07-19T18:00:37.879Z apply ops=1 removed=0
2026-07-19T18:00:52.806Z consolidate trigger=turn-close digest=0 ops=0 [redacted] reason=duplicate duplicateOf=project.md:px4_emergency_tools
2026-07-19T18:01:22.375Z regenerate index.kmem bytes=2842 [redacted]
2026-07-19T18:01:22.376Z apply ops=1 removed=0

## decisions.jsonl
{"v":1,"time":"2026-07-19T17:57:58.136Z","kind":"log","result":"logged","summary":"enable project source=command"}
{"v":1,"time":"2026-07-19T17:57:58.156Z","kind":"log","result":"logged","summary":"regenerate index.kmem bytes=0 [redacted]"}
{"v":1,"time":"2026-07-19T17:58:01.372Z","kind":"log","result":"logged","summary":"regenerate index.kmem bytes=0 [redacted]"}
{"v":1,"time":"2026-07-19T17:58:20.480Z","kind":"log","result":"logged","summary":"regenerate index.kmem bytes=626 [redacted]"}
{"v":1,"time":"2026-07-19T17:58:20.481Z","kind":"log","result":"logged","summary":"session digest session=ses_084c789e5ffe19qdYlxvTv9C7t [redacted] indexTokens=118"}
{"v":1,"time":"2026-07-19T17:58:20.517Z","kind":"digest","trigger":"turn-close","sessionID":"ses_084c789e5ffe19qdYlxvTv9C7t","result":"saved","llm":true,"parsed":true,"fallback":false,"tokens":559,"operationCount":1,"skippedCount":0,"summary":"session digest saved"}
{"v":1,"time":"2026-07-19T17:58:20.550Z","kind":"typed","trigger":"turn-close","sessionID":"ses_084c789e5ffe19qdYlxvTv9C7t","result":"skipped","llm":true,"parsed":true,"fallback":false,"tokens":3351,"operationCount":0,"skippedCount":3,"skipped":[{"reason":"in_progress","text":"EKF2 调试和 preflight Z 一致性检查的调试会话，尚未确认为稳定决策","file":"project.md","section":"Decisions"},{"reason":"transient","text":"飞控 reboot 命令和重连状态","file":"environment.md","section":"Commands"},{"reason":"in_progress","text":"MAVROS 通信中断的排查（尚未确认根因）","file":"project.md","section":"Open Questions"}],"operations":[],"files":[],"summary":"typed consolidation skipped 3 candidates"}
{"v":1,"time":"2026-07-19T17:58:20.566Z","kind":"log","result":"logged","summary":"consolidate trigger=turn-close digest=1 ops=0 [redacted] reason=in_progress"}
{"v":1,"time":"2026-07-19T17:59:25.119Z","kind":"log","result":"logged","summary":"regenerate index.kmem bytes=1235 [redacted]"}
{"v":1,"time":"2026-07-19T17:59:25.131Z","kind":"log","result":"logged","summary":"apply ops=1 removed=0"}
{"v":1,"time":"2026-07-19T17:59:25.136Z","kind":"typed","trigger":"explicit","sessionID":"ses_084c789e5ffe19qdYlxvTv9C7t","result":"saved","llm":false,"parsed":true,"fallback":false,"tokens":0,"operationCount":1,"skippedCount":0,"skipped":[],"operations":[{"action":"add","key":"px4_emergency_tools"}],"files":[],"summary":"explicit memory operation saved 1 ops"}
{"v":1,"time":"2026-07-19T17:59:45.109Z","kind":"log","result":"logged","summary":"regenerate index.kmem bytes=1235 [redacted]"}
{"v":1,"time":"2026-07-19T17:59:45.122Z","kind":"log","result":"logged","summary":"apply ops=0 removed=0"}
{"v":1,"time":"2026-07-19T17:59:54.202Z","kind":"log","result":"logged","summary":"regenerate index.kmem bytes=1701 [redacted]"}
{"v":1,"time":"2026-07-19T17:59:54.204Z","kind":"log","result":"logged","summary":"apply ops=1 removed=0"}
{"v":1,"time":"2026-07-19T17:59:54.218Z","kind":"typed","trigger":"explicit","sessionID":"ses_084c789e5ffe19qdYlxvTv9C7t","result":"saved","llm":false,"parsed":true,"fallback":false,"tokens":0,"operationCount":1,"skippedCount":0,"skipped":[],"operations":[{"action":"add","key":"px4_test_tools_workspace"}],"files":[],"summary":"explicit memory operation saved 1 ops"}
{"v":1,"time":"2026-07-19T18:00:11.754Z","kind":"log","result":"logged","summary":"regenerate index.kmem bytes=2129 [redacted]"}
{"v":1,"time":"2026-07-19T18:00:11.756Z","kind":"log","result":"logged","summary":"apply ops=1 removed=0"}
{"v":1,"time":"2026-07-19T18:00:11.771Z","kind":"typed","trigger":"explicit","sessionID":"ses_084c789e5ffe19qdYlxvTv9C7t","result":"saved","llm":false,"parsed":true,"fallback":false,"tokens":0,"operationCount":1,"skippedCount":0,"skipped":[],"operations":[{"action":"add","key":"mavros_xyz_offboard_design"}],"files":[],"summary":"explicit memory operation saved 1 ops"}
{"v":1,"time":"2026-07-19T18:00:20.468Z","kind":"typed","trigger":"turn-close","sessionID":"ses_084c789e5ffe19qdYlxvTv9C7t","result":"skipped","llm":false,"parsed":false,"fallback":false,"reason":"interval","tokens":0,"operationCount":0,"skippedCount":1,"summary":"memory capture skipped: interval"}
{"v":1,"time":"2026-07-19T18:00:37.866Z","kind":"log","result":"logged","summary":"regenerate index.kmem bytes=2294 [redacted]"}
{"v":1,"time":"2026-07-19T18:00:37.879Z","kind":"log","result":"logged","summary":"apply ops=1 removed=0"}
{"v":1,"time":"2026-07-19T18:00:37.882Z","kind":"typed","trigger":"explicit","sessionID":"ses_084c789e5ffe19qdYlxvTv9C7t","result":"saved","llm":false,"parsed":true,"fallback":false,"tokens":0,"operationCount":1,"skippedCount":0,"skipped":[],"operations":[{"action":"add","key":"electroniccompetion"}],"files":[],"summary":"explicit memory operation saved 1 ops"}
{"v":1,"time":"2026-07-19T18:00:52.802Z","kind":"typed","trigger":"turn-close","sessionID":"ses_084c789e5ffe19qdYlxvTv9C7t","result":"skipped","llm":true,"parsed":true,"fallback":false,"tokens":2873,"operationCount":0,"skippedCount":1,"skipped":[{"reason":"duplicate","text":"本次用到的工具已记录在 px4_emergency_tools、px4_test_tools_workspace、mavros_xyz_offboard_design 中","duplicateOf":"project.md:px4_emergency_tools","file":"project.md","section":"Facts"}],"operations":[],"files":[],"summary":"typed consolidation skipped 1 candidates"}
{"v":1,"time":"2026-07-19T18:00:52.806Z","kind":"log","result":"logged","summary":"consolidate trigger=turn-close digest=0 ops=0 [redacted] reason=duplicate duplicateOf=project.md:px4_emergency_tools"}
{"v":1,"time":"2026-07-19T18:01:22.375Z","kind":"log","result":"logged","summary":"regenerate index.kmem bytes=2842 [redacted]"}
{"v":1,"time":"2026-07-19T18:01:22.376Z","kind":"log","result":"logged","summary":"apply ops=1 removed=0"}
{"v":1,"time":"2026-07-19T18:01:22.393Z","kind":"typed","trigger":"explicit","sessionID":"ses_084c789e5ffe19qdYlxvTv9C7t","result":"saved","llm":false,"parsed":true,"fallback":false,"tokens":0,"operationCount":1,"skippedCount":0,"skipped":[],"operations":[{"action":"add","key":"px4_ekf_optical_flow_fusion_params"}],"files":[],"summary":"explicit memory operation saved 1 ops"}
{"v":1,"time":"2026-07-19T18:01:50.328Z","kind":"typed","trigger":"turn-close","sessionID":"ses_084c789e5ffe19qdYlxvTv9C7t","result":"skipped","llm":false,"parsed":false,"fallback":false,"reason":"interval","tokens":0,"operationCount":0,"skippedCount":1,"summary":"memory capture skipped: interval"}
