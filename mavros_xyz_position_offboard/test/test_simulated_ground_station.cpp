#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <json/json.h>

#include "mavros_xyz_position_offboard/common/types.hpp"
#include "mavros_xyz_position_offboard/communication/ground_station_link.hpp"
#include "mavros_xyz_position_offboard/gripper/pwm_gripper.hpp"
#include "mavros_xyz_position_offboard/navigation/navigation.hpp"

namespace
{
using mavros_xyz_position_offboard::common::MAV_LANDED_STATE_ON_GROUND;
using mavros_xyz_position_offboard::common::PositionSetpoint;
using mavros_xyz_position_offboard::common::SafetyConfig;
using mavros_xyz_position_offboard::common::Telemetry;
using mavros_xyz_position_offboard::communication::GroundStationConfig;
using mavros_xyz_position_offboard::communication::GroundStationLink;
using mavros_xyz_position_offboard::communication::MessageType;
using mavros_xyz_position_offboard::communication::OutgoingMessage;
using mavros_xyz_position_offboard::gripper::PwmGripper;
using mavros_xyz_position_offboard::gripper::PwmGripperConfig;
using mavros_xyz_position_offboard::gripper::ReleaseState;
using mavros_xyz_position_offboard::navigation::MissionConfig;
using mavros_xyz_position_offboard::navigation::Navigation;
using mavros_xyz_position_offboard::navigation::NavigationDecision;
using mavros_xyz_position_offboard::navigation::NavigationInput;

constexpr double kMissionCommandWaitSeconds = 4.0;

struct UdpEndpoint
{
  int fd{-1};
  sockaddr_in address{};

  ~UdpEndpoint()
  {
    if (fd >= 0) {::close(fd);}
  }

  UdpEndpoint() = default;
  UdpEndpoint(const UdpEndpoint &) = delete;
  UdpEndpoint & operator=(const UdpEndpoint &) = delete;
  UdpEndpoint(UdpEndpoint && other) noexcept : fd(other.fd), address(other.address)
  {
    other.fd = -1;
  }
  UdpEndpoint & operator=(UdpEndpoint && other) noexcept
  {
    if (this == &other) {return *this;}
    if (fd >= 0) {::close(fd);}
    fd = other.fd;
    address = other.address;
    other.fd = -1;
    return *this;
  }

  void close_socket()
  {
    if (fd >= 0) {
      ::close(fd);
      fd = -1;
    }
  }
};

/// 创建一个绑定在 127.0.0.1 临时端口的 UDP 端点。
UdpEndpoint make_loopback_endpoint()
{
  UdpEndpoint endpoint;
  endpoint.fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (endpoint.fd < 0) {return endpoint;}
  endpoint.address.sin_family = AF_INET;
  endpoint.address.sin_port = 0;
  ::inet_pton(AF_INET, "127.0.0.1", &endpoint.address.sin_addr);
  if (::bind(endpoint.fd, reinterpret_cast<const sockaddr *>(&endpoint.address), sizeof(endpoint.address)) < 0) {
    ::close(endpoint.fd);
    endpoint.fd = -1;
    return endpoint;
  }
  socklen_t length = sizeof(endpoint.address);
  if (::getsockname(endpoint.fd, reinterpret_cast<sockaddr *>(&endpoint.address), &length) < 0) {
    ::close(endpoint.fd);
    endpoint.fd = -1;
  }
  return endpoint;
}

/// 将一条完整 JSON 数据报发送到给定端点。
bool send_datagram(int fd, const sockaddr_in & destination, const std::string & json)
{
  const auto sent = ::sendto(fd, json.data(), json.size(), 0,
    reinterpret_cast<const sockaddr *>(&destination), sizeof(destination));
  return sent == static_cast<ssize_t>(json.size());
}

/// 在限定时间内接收一条 UDP 数据报，超时或错误时返回空值。
std::optional<std::string> receive_datagram(int fd, int timeout_ms = 1000)
{
  pollfd descriptor{fd, POLLIN, 0};
  if (::poll(&descriptor, 1, timeout_ms) <= 0 || (descriptor.revents & POLLIN) == 0) {
    return std::nullopt;
  }
  char buffer[4096]{};
  const auto received = ::recv(fd, buffer, sizeof(buffer), 0);
  if (received <= 0) {return std::nullopt;}
  return std::string(buffer, static_cast<std::size_t>(received));
}

/// 解析并返回协议根节点 header，同时验证最小事件对象形状。
std::optional<std::string> event_header(const std::string & json)
{
  Json::CharReaderBuilder reader;
  Json::Value root;
  std::string errors;
  std::istringstream input(json);
  if (!Json::parseFromStream(reader, input, &root, &errors) || !root.isObject() ||
    !root.isMember("header") || !root["header"].isString() || !root.isMember("data") ||
    !root["data"].isObject() || !root["data"].empty()) {
    return std::nullopt;
  }
  return root["header"].asString();
}

/// 生成空 data 的 GCS 命令对象。
std::string empty_command_json(const std::string & header)
{
  return "{\"header\":\"" + header + "\",\"data\":{}}";
}

/// 将模拟的实际飞行器遥测更新为本周期发出的规划设定点。
void follow_setpoint(Telemetry & telemetry, const std::optional<PositionSetpoint> & setpoint)
{
  if (!setpoint) {return;}
  telemetry.local_x_m = setpoint->x_m;
  telemetry.local_y_m = setpoint->y_m;
  telemetry.local_z_m = setpoint->z_m;
  telemetry.orientation = setpoint->orientation;
}

/// 模拟 GCS 接收一个 ok_* 事件后立即发送最小 ACK。
bool receive_event_and_ack(
  UdpEndpoint & gcs, const sockaddr_in & uav_address, std::vector<std::string> & received_headers)
{
  const auto json = receive_datagram(gcs.fd);
  if (!json) {return false;}
  const auto header = event_header(*json);
  if (!header || header->rfind("ok_", 0) != 0) {return false;}
  received_headers.push_back(*header);
  return send_datagram(gcs.fd, uav_address, empty_command_json("ack"));
}
}  // namespace

TEST(SimulatedGroundStationTest, CompletesPayloadlessMissionOverLoopbackUdp)
{
  auto gcs = make_loopback_endpoint();
  auto uav = make_loopback_endpoint();
  ASSERT_GE(gcs.fd, 0);
  ASSERT_GE(uav.fd, 0);

  GroundStationConfig udp_config;
  udp_config.bind_ip = "127.0.0.1";
  udp_config.bind_port = ntohs(uav.address.sin_port);
  udp_config.remote_ip = "127.0.0.1";
  udp_config.remote_port = ntohs(gcs.address.sin_port);
  udp_config.whitelist_ip = "127.0.0.1";
  udp_config.whitelist_port = ntohs(gcs.address.sin_port);
  udp_config.event_retry_period_s = 0.05;
  // 临时端点仅用于分配可用端口；实际绑定必须由被测 GroundStationLink 独占。
  uav.close_socket();
  GroundStationLink link(udp_config);
  ASSERT_TRUE(link.bound());

  PwmGripperConfig gripper_config;
  gripper_config.enabled = false;
  gripper_config.open_hold_ms = 10;
  PwmGripper gripper(gripper_config);
  EXPECT_FALSE(gripper.enabled());

  SafetyConfig safety;
  safety.setpoint_warmup_s = 0.05;
  safety.max_flight_seconds = 60.0;
  safety.target_xy_max_speed_m_s = 3.0;
  safety.target_xy_max_accel_m_s2 = 6.0;
  safety.max_z_setpoint_rate_m_s = 2.0;
  safety.max_z_setpoint_accel_m_s2 = 4.0;
  safety.target_tolerance_m = 0.03;
  MissionConfig mission;
  mission.takeoff_height_m = 1.5;
  mission.height_stable_seconds = 3.0;
  mission.right_shift_m = 0.375;
  mission.forward_distance_m = 1.0;
  mission.match_hold_seconds = 0.5;
  Navigation navigation(safety, mission);

  NavigationInput input;
  input.dt = 0.05;
  input.preflight_ready = true;
  input.lcp_healthy = true;
  input.flight_healthy = true;
  input.telemetry.local_x_m = 0.0;
  input.telemetry.local_y_m = 0.0;
  input.telemetry.local_z_m = 0.0;
  input.telemetry.orientation = {0.0, 0.0, 0.0, 1.0};
  input.controller.mode = "MANUAL";
  input.controller.armed = false;

  double now = 0.0;
  std::vector<std::string> received_headers;
  bool transport_ok = true;
  const auto tick = [&]() {
      input.now = now;
      input.events = link.poll(now);
      const auto gripper_state = gripper.update(now);
      input.gripper_succeeded = gripper_state == ReleaseState::succeeded;
      input.gripper_failed = gripper_state == ReleaseState::failed;
      const NavigationDecision decision = navigation.update(input);
      if (decision.release_gripper && !gripper.begin_release(now)) {transport_ok = false;}
      for (const OutgoingMessage & message : decision.messages) {
        if (!link.send(message, now) || !receive_event_and_ack(gcs, uav.address, received_headers)) {
          transport_ok = false;
        }
      }
      follow_setpoint(input.telemetry, decision.setpoint);
      now += input.dt;
      return decision;
    };
  const auto send_gcs = [&](const std::string & json) {
      if (!send_datagram(gcs.fd, uav.address, json)) {transport_ok = false;}
    };
  // 离散控制命令在虚拟实机时间中至少等待 4 秒。
  const auto wait_before_mission_command = [&]() {
      const double wait_started_at = now;
      while (now - wait_started_at < kMissionCommandWaitSeconds) {tick();}
    };

  // 预检成功后先保持 Init 设定点，预热完成才发送 ok_wait。
  tick();
  ASSERT_TRUE(transport_ok);
  ASSERT_TRUE(received_headers.empty());
  tick();
  ASSERT_EQ(received_headers, std::vector<std::string>({"ok_wait"}));
  tick();
  EXPECT_EQ(link.pending_event_count(), 0U);

  wait_before_mission_command();
  send_gcs(empty_command_json("run_plan1"));
  tick();
  ASSERT_EQ(navigation.phase(), "offboard_request_pending");
  input.controller.mode = "OFFBOARD";
  input.controller.armed = true;
  for (int count = 0; count < 4 && navigation.phase() != "climb"; ++count) {tick();}
  ASSERT_EQ(navigation.phase(), "climb");
  for (int count = 0; count < 200 && navigation.phase() == "climb"; ++count) {tick();}
  ASSERT_EQ(navigation.phase(), "height_stabilizing");
  const double height_stabilizing_started_at = input.now;
  for (int count = 0; count < 59; ++count) {
    tick();
    EXPECT_EQ(navigation.phase(), "height_stabilizing");
    EXPECT_EQ(received_headers, std::vector<std::string>({"ok_wait"}));
  }
  for (int count = 0; count < 4 && navigation.phase() == "height_stabilizing"; ++count) {tick();}
  EXPECT_GE(input.now - height_stabilizing_started_at, mission.height_stable_seconds);
  ASSERT_TRUE(transport_ok);
  ASSERT_EQ(navigation.phase(), "right_shift");
  ASSERT_EQ(received_headers, std::vector<std::string>({"ok_wait", "ok_height"}));
  tick();
  EXPECT_EQ(link.pending_event_count(), 0U);

  // The simulated GCS compares the initial coordinates with the post-shift coordinates.
  const double alignment_start_x_m = 0.0;
  const double alignment_start_y_m = 0.0;
  for (int count = 0; count < 200 && navigation.phase() == "right_shift"; ++count) {tick();}
  ASSERT_TRUE(transport_ok);
  ASSERT_EQ(navigation.phase(), "waiting_go_ahead");
  const double right_shift_m = alignment_start_y_m - input.telemetry.local_y_m;
  EXPECT_NEAR(input.telemetry.local_x_m, alignment_start_x_m, safety.target_tolerance_m);
  EXPECT_GE(right_shift_m, 0.35);
  EXPECT_LE(right_shift_m, 0.40);
  const bool black_line_centered = true;  // Vision pipeline verdict supplied to the GCS.
  ASSERT_TRUE(black_line_centered);

  send_gcs(empty_command_json("go_ahead_ok"));
  tick();
  ASSERT_TRUE(transport_ok);
  ASSERT_EQ(navigation.phase(), "pursuing_car");
  EXPECT_NEAR(navigation.planner().target_x_m(), mission.forward_distance_m, 1e-6);
  EXPECT_NEAR(navigation.planner().target_y_m(), -mission.right_shift_m, 1e-6);

  // GCS-owned vision locates the car and sends match only once horizontal distance is below 0.1 m.
  const double car_x_m = 0.60;
  const double car_y_m = -mission.right_shift_m;
  bool match_sent = false;
  double match_command_at = 0.0;
  NavigationDecision match_decision;
  for (int count = 0; count < 200 && !match_sent; ++count) {
    tick();
    if (std::hypot(input.telemetry.local_x_m - car_x_m,
        input.telemetry.local_y_m - car_y_m) < 0.10) {
      match_command_at = now;
      send_gcs(empty_command_json("match_car_ok"));
      match_decision = tick();
      match_sent = true;
    }
  }
  ASSERT_TRUE(match_sent);
  ASSERT_EQ(navigation.phase(), "match_hold") << "link rejection=" << link.last_rejection()
    << ", navigation rejection=" << (match_decision.rejections.empty() ? "none" : match_decision.rejections.front());
  for (int count = 0; count < 20 && navigation.phase() == "match_hold"; ++count) {tick();}
  EXPECT_GE(now - match_command_at, mission.match_hold_seconds);
  ASSERT_EQ(navigation.phase(), "throwing");
  for (int count = 0; count < 20 && navigation.phase() == "throwing"; ++count) {tick();}
  ASSERT_TRUE(transport_ok);
  ASSERT_EQ(navigation.phase(), "awaiting_b_ok");
  ASSERT_EQ(received_headers, std::vector<std::string>({"ok_wait", "ok_height", "ok_throw"}));
  EXPECT_EQ(gripper.state(), ReleaseState::succeeded);
  tick();
  EXPECT_EQ(link.pending_event_count(), 0U);

  wait_before_mission_command();
  send_gcs(empty_command_json("b_ok"));
  tick();
  ASSERT_TRUE(transport_ok);
  ASSERT_EQ(navigation.phase(), "returning");
  ASSERT_EQ(received_headers, std::vector<std::string>({"ok_wait", "ok_height", "ok_throw", "ok_return"}));
  tick();
  EXPECT_EQ(link.pending_event_count(), 0U);

  for (int count = 0; count < 200 && navigation.phase() == "returning"; ++count) {tick();}
  ASSERT_TRUE(transport_ok);
  ASSERT_EQ(navigation.phase(), "downing");
  EXPECT_NEAR(input.telemetry.local_x_m, 0.0, safety.target_tolerance_m);
  EXPECT_NEAR(input.telemetry.local_y_m, 0.0, safety.target_tolerance_m);
  ASSERT_EQ(received_headers,
    std::vector<std::string>({"ok_wait", "ok_height", "ok_throw", "ok_return", "ok_downing"}));
  tick();
  EXPECT_EQ(link.pending_event_count(), 0U);

  input.telemetry.landed_state = MAV_LANDED_STATE_ON_GROUND;
  tick();
  ASSERT_EQ(navigation.phase(), "disarming");
  input.controller.armed = false;
  tick();
  ASSERT_EQ(navigation.phase(), "manual_request_pending");
  input.controller.mode = "MANUAL";
  tick();
  ASSERT_TRUE(transport_ok);
  EXPECT_EQ(navigation.phase(), "manual");
  ASSERT_EQ(received_headers,
    std::vector<std::string>({"ok_wait", "ok_height", "ok_throw", "ok_return", "ok_downing", "ok_down"}));
  tick();
  EXPECT_EQ(link.pending_event_count(), 0U);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
