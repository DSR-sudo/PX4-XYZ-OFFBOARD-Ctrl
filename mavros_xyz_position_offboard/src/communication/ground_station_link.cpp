#include "mavros_xyz_position_offboard/communication/ground_station_link.hpp"

#include "mavros_xyz_position_offboard/common/artifact_log.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <json/json.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace mavros_xyz_position_offboard::communication
{
namespace
{
/// 判断整数是否位于合法 UDP 端口范围内。
bool valid_port(int value) {return value >= 1 && value <= 65535;}

/// 判断字符串是否为不依赖 DNS 解析的 IPv4 字面量。
bool valid_ipv4(const std::string & value)
{
  in_addr address{};
  return ::inet_pton(AF_INET, value.c_str(), &address) == 1;
}

/// 严格校验字节序列是否为合法 UTF-8，拒绝过长编码和代理项。
bool valid_utf8(const std::string & text)
{
  const auto * bytes = reinterpret_cast<const unsigned char *>(text.data());
  for (std::size_t i = 0; i < text.size();) {
    const unsigned char first = bytes[i];
    if (first <= 0x7fU) {++i; continue;}
    std::size_t length = 0;
    std::uint32_t codepoint = 0;
    if ((first & 0xe0U) == 0xc0U) {length = 2; codepoint = first & 0x1fU;}
    else if ((first & 0xf0U) == 0xe0U) {length = 3; codepoint = first & 0x0fU;}
    else if ((first & 0xf8U) == 0xf0U) {length = 4; codepoint = first & 0x07U;}
    else {return false;}
    if (i + length > text.size()) {return false;}
    for (std::size_t offset = 1; offset < length; ++offset) {
      if ((bytes[i + offset] & 0xc0U) != 0x80U) {return false;}
      codepoint = (codepoint << 6U) | (bytes[i + offset] & 0x3fU);
    }
    if ((length == 2 && codepoint < 0x80U) || (length == 3 && codepoint < 0x800U) ||
      (length == 4 && codepoint < 0x10000U) || codepoint > 0x10ffffU ||
      (codepoint >= 0xd800U && codepoint <= 0xdfffU)) {return false;}
    i += length;
  }
  return true;
}

/// 判断 JSON 对象是否只包含给定消息类型允许的成员。
bool only_members(const Json::Value & value, const std::vector<std::string> & allowed)
{
  for (const auto & name : value.getMemberNames()) {
    bool found = false;
    for (const auto & candidate : allowed) {
      if (name == candidate) {found = true; break;}
    }
    if (!found) {return false;}
  }
  return true;
}

/// 判断 JSON 值是否为有限数值。
bool finite_number(const Json::Value & value)
{
  return value.isNumeric() && std::isfinite(value.asDouble());
}

/// 将有限浮点值编码为 JSON 数值文本。
std::string number(double value)
{
  if (!std::isfinite(value)) {throw std::invalid_argument("outgoing JSON number is non-finite");}
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  return stream.str();
}

/// 将可选有限浮点值编码为 JSON 数值或 null。
std::string optional_number(const std::optional<double> & value)
{
  return value ? number(*value) : "null";
}

/// 将 ROS 消息头编码为 xyzstatus 使用的 JSON 对象。
std::string ros_header_json(const RosHeader & header)
{
  std::ostringstream stream;
  stream << "{\"stamp\":{\"sec\":" << header.stamp_sec << ",\"nanosec\":" << header.stamp_nanosec
         << "},\"frame_id\":\"" << common::json_escape(header.frame_id) << "\"}";
  return stream.str();
}

/// 将可选 ROS 时间戳编码为 JSON 对象或 null。
std::string optional_stamp_json(const std::optional<common::RosTimestamp> & stamp)
{
  if (!stamp) {return "null";}
  return "{\"sec\":" + std::to_string(stamp->sec) + ",\"nanosec\":" +
    std::to_string(stamp->nanosec) + "}";
}

/// 检查 xyzstatus 中所有必须存在的数值字段是否有限。
bool finite_xyz_status(const XyzStatus & value)
{
  const double numbers[] = {
    value.position_x_m, value.position_y_m, value.yaw_rad, value.front_distance_m,
    value.rear_distance_m, value.left_distance_m, value.right_distance_m,
    value.map_size_x_m, value.map_size_y_m};
  for (const double number_value : numbers) {
    if (!std::isfinite(number_value)) {return false;}
  }
  return !value.position_z_m || std::isfinite(*value.position_z_m);
}
}  // 匿名命名空间

/// 将协议消息枚举转换为线上的 header 字符串。
std::string to_string(MessageType type)
{
  switch (type) {
    case MessageType::run_plan1: return "run_plan1";
    case MessageType::car_status: return "car_status";
    case MessageType::match_car_ok: return "match_car_ok";
    case MessageType::b_ok: return "b_ok";
    case MessageType::ack: return "ack";
    case MessageType::ok_wait: return "ok_wait";
    case MessageType::ok_height: return "ok_height";
    case MessageType::ok_throw: return "ok_throw";
    case MessageType::ok_return: return "ok_return";
    case MessageType::ok_downing: return "ok_downing";
    case MessageType::ok_down: return "ok_down";
    case MessageType::xyzstatus: return "xyzstatus";
    default: return "invalid";
  }
}

/// 判断消息是否属于需要 ACK 的有序离散事件。
bool is_discrete_event(MessageType type)
{
  return type == MessageType::ok_wait || type == MessageType::ok_height ||
         type == MessageType::ok_throw || type == MessageType::ok_return ||
         type == MessageType::ok_downing || type == MessageType::ok_down;
}

/// 校验 UDP 端点、重传周期和跟车距离上限配置。
void GroundStationConfig::validate() const
{
  if (!valid_ipv4(bind_ip) || !valid_ipv4(remote_ip) || !valid_ipv4(whitelist_ip)) {
    throw std::invalid_argument("UDP addresses must be IPv4 literals");
  }
  if (!valid_port(bind_port) || !valid_port(remote_port) || !valid_port(whitelist_port)) {
    throw std::invalid_argument("UDP ports must be within 1..65535");
  }
  if (!std::isfinite(event_retry_period_s) || !std::isfinite(max_tracking_distance_m) ||
    event_retry_period_s <= 0.0 || max_tracking_distance_m <= 0.0) {
    throw std::invalid_argument("UDP retry period and tracking distance must be finite and positive");
  }
}

/// 创建、设为非阻塞并绑定 UDP 套接字；失败原因记录在 bind_error_。
GroundStationLink::GroundStationLink(const GroundStationConfig & config) : config_(config)
{
  config_.validate();
  if (!config_.enabled) {return;}
  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {bind_error_ = std::strerror(errno); return;}
  const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    bind_error_ = std::strerror(errno); ::close(socket_fd_); socket_fd_ = -1; return;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<std::uint16_t>(config_.bind_port));
  ::inet_pton(AF_INET, config_.bind_ip.c_str(), &address.sin_addr);
  if (::bind(socket_fd_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
    bind_error_ = std::strerror(errno); ::close(socket_fd_); socket_fd_ = -1;
  }
}

/// 关闭仍处于打开状态的 UDP 套接字。
GroundStationLink::~GroundStationLink() {if (socket_fd_ >= 0) {::close(socket_fd_);}}

/// 构造被拒绝的协议事件并记录拒绝原因。
ProtocolEvent GroundStationLink::reject(const std::string & reason, double now)
{
  last_rejection_ = reason;
  ProtocolEvent event;
  event.received_at = now;
  event.rejection_reason = reason;
  return event;
}

/// 确认并移除队列中最早的未确认离散事件。
void GroundStationLink::acknowledge_earliest()
{
  if (!pending_events_.empty()) {pending_events_.pop_front();}
}

/// 校验并解析一条 GCS 到 UAV 的 JSON 数据报，必要时推进 ACK 队列。
ProtocolEvent GroundStationLink::decode_datagram(
  const std::string & payload, const std::string & source_ip, int source_port, double now)
{
  if (source_ip != config_.whitelist_ip || source_port != config_.whitelist_port) {
    return reject("source_not_whitelisted", now);
  }
  if (!std::isfinite(now)) {return reject("invalid_receive_time", now);}
  if (!valid_utf8(payload)) {return reject("invalid_utf8", now);}
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  builder["failIfExtra"] = true;
  builder["rejectDupKeys"] = true;
  Json::Value root;
  std::string errors;
  std::istringstream input(payload);
  if (!Json::parseFromStream(builder, input, &root, &errors) || !root.isObject()) {
    return reject("invalid_json", now);
  }
  if (!only_members(root, {"header", "data"}) || !root.isMember("header") ||
    !root["header"].isString() || !root.isMember("data") || !root["data"].isObject()) {
    return reject("invalid_envelope", now);
  }

  ProtocolEvent event;
  event.received_at = now;
  const std::string header = root["header"].asString();
  const auto & data = root["data"];
  if (header == "run_plan1" || header == "match_car_ok" || header == "b_ok" || header == "ack") {
    if (!data.empty()) {return reject("nonempty_event_data", now);}
    event.type = header == "run_plan1" ? MessageType::run_plan1 :
      (header == "match_car_ok" ? MessageType::match_car_ok :
      (header == "b_ok" ? MessageType::b_ok : MessageType::ack));
    if (event.type == MessageType::ack) {acknowledge_earliest();}
  } else if (header == "car_status") {
    if (!only_members(data, {"distance", "angle"}) || !data.isMember("distance") ||
      !data.isMember("angle") || !finite_number(data["distance"]) || !finite_number(data["angle"])) {
      return reject("invalid_car_status_data", now);
    }
    const double distance = data["distance"].asDouble();
    const double angle = data["angle"].asDouble();
    if (distance < 0.0 || distance > config_.max_tracking_distance_m || angle < -180.0 || angle > 180.0) {
      return reject("car_status_out_of_range", now);
    }
    event.type = MessageType::car_status;
    event.car_status = CarStatus{distance, angle};
  } else {
    return reject("unknown_or_wrong_direction_header", now);
  }
  event.accepted = true;
  last_rejection_.clear();
  return event;
}

/// 非阻塞读取当前可用的数据报，并逐条解析为协议事件。
std::vector<ProtocolEvent> GroundStationLink::poll(double now)
{
  std::vector<ProtocolEvent> result;
  if (socket_fd_ < 0) {return result;}
  for (int count = 0; count < 64; ++count) {
    char buffer[65536];
    sockaddr_in source{};
    socklen_t source_length = sizeof(source);
    const auto received = ::recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
      reinterpret_cast<sockaddr *>(&source), &source_length);
    if (received < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        last_rejection_ = std::string("recv_error:") + std::strerror(errno);
      }
      break;
    }
    char address[INET_ADDRSTRLEN]{};
    ::inet_ntop(AF_INET, &source.sin_addr, address, sizeof(address));
    result.push_back(decode_datagram(
      std::string(buffer, static_cast<std::size_t>(received)), address,
      ntohs(source.sin_port), now));
  }
  return result;
}

/// 将需确认的 ok_* 离散事件编码为固定协议对象。
std::string GroundStationLink::encode(const OutgoingMessage & message) const
{
  if (!is_discrete_event(message.type)) {
    throw std::invalid_argument("only discrete ok_* messages may use the event queue");
  }
  return "{\"header\":\"" + to_string(message.type) + "\",\"data\":{}}";
}

/// 编码包含 LCP 数据和 Z 元数据的完整 xyzstatus 消息。
std::string GroundStationLink::encode_xyzstatus(const XyzStatus & status) const
{
  if (!finite_xyz_status(status)) {throw std::invalid_argument("xyzstatus contains a non-finite LCP field");}
  if (!status.z_valid && (status.position_z_m || status.z_source != "none" || status.z_source_stamp || status.z_quality)) {
    throw std::invalid_argument("invalid Z must use the required null/none representation");
  }
  if (status.z_valid && (!status.position_z_m || status.z_source == "none" || !status.z_source_stamp)) {
    throw std::invalid_argument("valid Z requires value, source, and source stamp");
  }
  std::ostringstream stream;
  stream << "{\"header\":\"xyzstatus\",\"data\":{"
         << "\"header\":" << ros_header_json(status.header)
         << ",\"status\":" << static_cast<unsigned int>(status.status)
         << ",\"map_locked\":" << (status.map_locked ? "true" : "false")
         << ",\"pose_valid\":" << (status.pose_valid ? "true" : "false")
         << ",\"position_x_m\":" << number(status.position_x_m)
         << ",\"position_y_m\":" << number(status.position_y_m)
         << ",\"position_z_m\":" << optional_number(status.position_z_m)
         << ",\"z_source\":\"" << common::json_escape(status.z_source) << "\""
         << ",\"z_source_stamp\":" << optional_stamp_json(status.z_source_stamp)
         << ",\"z_quality\":" << optional_number(status.z_quality)
         << ",\"z_valid\":" << (status.z_valid ? "true" : "false")
         << ",\"yaw_rad\":" << number(status.yaw_rad)
         << ",\"front_distance_m\":" << number(status.front_distance_m)
         << ",\"rear_distance_m\":" << number(status.rear_distance_m)
         << ",\"left_distance_m\":" << number(status.left_distance_m)
         << ",\"right_distance_m\":" << number(status.right_distance_m)
         << ",\"map_size_x_m\":" << number(status.map_size_x_m)
         << ",\"map_size_y_m\":" << number(status.map_size_y_m) << "}}";
  return stream.str();
}

/// 使用非阻塞 UDP 将完整 JSON 数据报发送到固定远端。
bool GroundStationLink::send_json(const std::string & json)
{
  if (socket_fd_ < 0 || json.empty()) {return false;}
  sockaddr_in remote{};
  remote.sin_family = AF_INET;
  remote.sin_port = htons(static_cast<std::uint16_t>(config_.remote_port));
  ::inet_pton(AF_INET, config_.remote_ip.c_str(), &remote.sin_addr);
  const auto sent = ::sendto(socket_fd_, json.data(), json.size(), MSG_DONTWAIT,
    reinterpret_cast<const sockaddr *>(&remote), sizeof(remote));
  return sent == static_cast<ssize_t>(json.size());
}

/// 将离散事件加入 ACK 队列，并立即尝试发送当前队首。
bool GroundStationLink::send(const OutgoingMessage & message, double now)
{
  if (!std::isfinite(now)) {throw std::invalid_argument("event send time must be finite");}
  pending_events_.push_back({encode(message), -std::numeric_limits<double>::infinity()});
  return retry_events(now);
}

/// 立即发送一条无需 ACK 的 xyzstatus 遥测消息。
bool GroundStationLink::send_xyzstatus(const XyzStatus & status)
{
  return send_json(encode_xyzstatus(status));
}

/// 到达重传周期时发送队首未确认离散事件。
bool GroundStationLink::retry_events(double now)
{
  if (!std::isfinite(now) || pending_events_.empty()) {return false;}
  auto & event = pending_events_.front();
  if (now - event.last_sent_at < config_.event_retry_period_s) {return false;}
  event.last_sent_at = now;
  return send_json(event.json);
}

/// 返回队首待确认事件的 JSON；队列为空时返回空值。
std::optional<std::string> GroundStationLink::pending_event_json() const
{
  if (pending_events_.empty()) {return std::nullopt;}
  return pending_events_.front().json;
}

}  // mavros_xyz_position_offboard::communication 命名空间
