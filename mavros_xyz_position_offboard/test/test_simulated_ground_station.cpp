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
using mavros_xyz_position_offboard::common::SafetyConfig;
using mavros_xyz_position_offboard::communication::GroundStationConfig;
using mavros_xyz_position_offboard::communication::GroundStationLink;
using mavros_xyz_position_offboard::communication::MessageType;
using mavros_xyz_position_offboard::communication::OutgoingMessage;
using mavros_xyz_position_offboard::gripper::PwmGripper;
using mavros_xyz_position_offboard::gripper::PwmGripperConfig;
using mavros_xyz_position_offboard::gripper::ReleaseState;
using mavros_xyz_position_offboard::navigation::MissionConfig;

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

}  // namespace

TEST(SimulatedGroundStationTest, QueuesOkBUntilTheGcsAcknowledgesItOverUdp)
{
  auto gcs = make_loopback_endpoint();
  auto uav = make_loopback_endpoint();
  ASSERT_GE(gcs.fd, 0);
  ASSERT_GE(uav.fd, 0);

  GroundStationConfig config;
  config.bind_ip = "127.0.0.1";
  config.bind_port = ntohs(uav.address.sin_port);
  config.remote_ip = "127.0.0.1";
  config.remote_port = ntohs(gcs.address.sin_port);
  config.whitelist_ip = "127.0.0.1";
  config.whitelist_port = ntohs(gcs.address.sin_port);
  config.event_retry_period_s = 0.01;
  uav.close_socket();
  GroundStationLink link(config);
  ASSERT_TRUE(link.bound());

  ASSERT_TRUE(link.send({MessageType::ok_b}, 1.0));
  const auto first = receive_datagram(gcs.fd);
  ASSERT_TRUE(first);
  ASSERT_EQ(event_header(*first), std::optional<std::string>("ok_b"));
  ASSERT_TRUE(send_datagram(gcs.fd, uav.address, empty_command_json("ack")));
  const auto events = link.poll(1.01);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_TRUE(events.front().accepted);
  EXPECT_EQ(link.pending_event_count(), 0U);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
