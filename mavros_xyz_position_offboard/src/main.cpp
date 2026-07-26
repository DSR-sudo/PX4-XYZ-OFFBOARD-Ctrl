#include <iostream>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "mavros_xyz_position_offboard/common/artifact_log.hpp"
#include "mavros_xyz_position_offboard/common/cli.hpp"
#include "mavros_xyz_position_offboard/offboard/offboard.hpp"

/// 剥离 ROS 参数、校验应用 CLI，并在单线程执行器中运行唯一节点。
int main(int argc, char ** argv)
{
  try {
    const auto application_arguments = rclcpp::remove_ros_arguments(argc, argv);
    for (const auto & argument : application_arguments) {
      if (argument == "--help" || argument == "-h") {
        std::cout << mavros_xyz_position_offboard::common::usage() << std::endl;
        return 0;
      }
    }
    const auto parsed = mavros_xyz_position_offboard::common::parse_options(application_arguments);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mavros_xyz_position_offboard::offboard::MavrosNativeXYZNode>(parsed.options, parsed.config);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "{\"schema\":\"px4.mavros_native_xyz.v1\",\"phase\":\"startup_error\",\"errors\":[\""
              << mavros_xyz_position_offboard::common::json_escape(error.what()) << "\"]}" << std::endl;
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 2;
  }
}
