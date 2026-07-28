#pragma once

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mavros_xyz_position_offboard/bridge/lcp_vision_bridge.hpp"
#include "mavros_xyz_position_offboard/common/artifact_log.hpp"
#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/communication/ground_station_link.hpp"
#include "mavros_xyz_position_offboard/initialization/initialization.hpp"
#include "mavros_xyz_position_offboard/navigation/navigation.hpp"
#include "mavros_xyz_position_offboard/offboard/offboard.hpp"

namespace mavros_xyz_position_offboard::application
{

class ApplicationNode final : public rclcpp::Node
{
public:
  /// 创建唯一 ROS 2 节点，并按固定依赖顺序组装初始化、通信、导航和飞控执行模块。
  ApplicationNode(const common::AppOptions & options, const common::SafetyConfig & config);
  /// 关闭应用持有的 artifact 日志并释放各模块资源。
  ~ApplicationNode() override;

private:
  /// 返回供控制周期、超时和日志使用的单调时钟秒数。
  static double monotonic_now();
  /// 声明并读取启动时生效的 UDP 端点、白名单、状态周期和安全包络参数。
  communication::GroundStationConfig load_ground_station_config();
  /// 按规定的六步顺序执行一次 20 Hz 单线程应用控制循环。
  void tick();
  /// 按日志周期写入统一 schema 状态并输出终端摘要。
  void emit_status(
    double now, const initialization::HealthSnapshot & health,
    const navigation::NavigationDecision & decision);

  const common::AppOptions options_;
  const common::SafetyConfig config_;
  initialization::Initialization initialization_;
  communication::GroundStationLink ground_station_;
  navigation::Navigation navigation_;
  offboard::Offboard offboard_;
  bridge::LcpVisionBridge lcp_vision_bridge_;
  common::ArtifactLogger artifact_log_;
  std::optional<double> last_tick_at_{};
  std::optional<double> sensor_fault_since_{};
  double last_log_at_{-1.0e100};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mavros_xyz_position_offboard::application
