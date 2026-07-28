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
#include <sstream>
#include <stdexcept>

namespace mavros_xyz_position_offboard::communication
{
namespace
{
/// 判断整数是否位于合法 UDP 端口范围。
bool valid_port(int value) {return value >= 1 && value <= 65535;}

/// 判断字符串是否为不依赖 DNS 的 IPv4 字面量。
bool valid_ipv4(const std::string & value)
{
  in_addr address{};
  return ::inet_pton(AF_INET, value.c_str(), &address) == 1;
}

/// 严格检查数据报字节序列是否为无过长编码和代理区字符的 UTF-8。
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

/// 判断 JSON 对象是否只包含当前消息类型允许的字段。
bool only_members(const Json::Value & value, const std::vector<std::string> & allowed)
{
  for (const auto & name : value.getMemberNames()) {
    bool found = false;
    for (const auto & candidate : allowed) {if (name == candidate) {found = true; break;}}
    if (!found) {return false;}
  }
  return true;
}

/// 判断 JSON 值是否为可转换成有限 double 的数字。
bool finite_number(const Json::Value & value)
{
  return value.isNumeric() && std::isfinite(value.asDouble());
}

/// 严格解析只含 x/y/z 三个有限数值字段的协议点。
bool decode_point(const Json::Value & value, Point3 & point)
{
  if (!value.isObject() || !only_members(value, {"x", "y", "z"}) ||
    !value.isMember("x") || !value.isMember("y") || !value.isMember("z") ||
    !finite_number(value["x"]) || !finite_number(value["y"]) || !finite_number(value["z"])) {return false;}
  point = {value["x"].asDouble(), value["y"].asDouble(), value["z"].asDouble()};
  return true;
}

/// 将有限浮点数编码为 JSON 数字，非有限数值编码为 null。
std::string number(double value)
{
  if (!std::isfinite(value)) {return "null";}
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  return stream.str();
}
}  // namespace

/// 精确比较三轴坐标，满足导航三包业务载荷必须完全一致的要求。
bool Point3::operator==(const Point3 & other) const
{
  return x_m == other.x_m && y_m == other.y_m && z_m == other.z_m;
}

/// 将协议枚举映射到 JSON V1 对外 type 名称。
std::string to_string(MessageType type)
{
  switch (type) {
    case MessageType::start: return "start";
    case MessageType::navigation_and_point: return "navigation_and_point";
    case MessageType::navigation_nfz: return "navigation_nfz";
    case MessageType::navigation_plan: return "navigation_plan";
    case MessageType::navigation_fly_plan_send_ok: return "navigation_fly_plan_send_ok";
    case MessageType::ack: return "ACK";
    case MessageType::ok_fly_plan_succeed: return "ok_fly_plan_succeed";
    case MessageType::ok_preflight: return "ok_preflight";
    case MessageType::ok_flight: return "ok_flight";
    case MessageType::wait_plan: return "wait_plan";
    case MessageType::ok_receive: return "ok_receive";
    case MessageType::ok_fly_plan: return "ok_fly_plan";
    case MessageType::xyz_state: return "xyz_state";
    case MessageType::battery_state: return "battery_state";
    default: return "invalid";
  }
}

/// 拒绝非法 IPv4、端口、非有限参数以及反向安全区间。
void GroundStationConfig::validate() const
{
  if (!valid_ipv4(bind_ip) || !valid_ipv4(remote_ip) || !valid_ipv4(whitelist_ip)) {
    throw std::invalid_argument("UDP addresses must be IPv4 literals");
  }
  if (!valid_port(bind_port) || !valid_port(remote_port) || !valid_port(whitelist_port)) {
    throw std::invalid_argument("UDP ports must be within 1..65535");
  }
  const double values[] = {status_period_s, relative_x_min_m, relative_x_max_m,
    relative_y_min_m, relative_y_max_m, absolute_z_min_m, absolute_z_max_m};
  for (const double value : values) {
    if (!std::isfinite(value)) {throw std::invalid_argument("UDP configuration must be finite");}
  }
  if (status_period_s <= 0.0 || relative_x_min_m > relative_x_max_m ||
    relative_y_min_m > relative_y_max_m || absolute_z_min_m > absolute_z_max_m) {
    throw std::invalid_argument("UDP status period or safety envelope is invalid");
  }
}

/// 在通信启用时创建、设置非阻塞并绑定 UDP socket，失败原因保存在状态中。
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

/// 关闭有效 socket，确保节点退出后释放绑定端口。
GroundStationLink::~GroundStationLink() {if (socket_fd_ >= 0) {::close(socket_fd_);}}

/// 生成未接受的协议事件并锁存稳定的通信拒绝原因。
ProtocolEvent GroundStationLink::reject(const std::string & reason, double now)
{
  last_rejection_ = reason;
  ProtocolEvent event;
  event.received_at = now;
  event.rejection_reason = reason;
  return event;
}

/// 验证来源、UTF-8、JSON V1 包络、序号、载荷和导航控制点安全包络。
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
  if (!root.isMember("version") || !root["version"].isInt() || root["version"].asInt() != 1) {
    return reject("invalid_version", now);
  }
  if (!root.isMember("type") || !root["type"].isString()) {return reject("invalid_type", now);}
  if (!root.isMember("seq") || !root["seq"].isUInt64()) {return reject("invalid_seq", now);}
  const auto seq = root["seq"].asUInt64();
  if (received_sequences_.count(seq) != 0U) {return reject("duplicate_seq", now);}
  received_sequences_.insert(seq);
  if (received_sequences_.size() > 4096U) {received_sequences_.clear(); received_sequences_.insert(seq);}

  ProtocolEvent event;
  event.seq = seq;
  event.received_at = now;
  const std::string type = root["type"].asString();
  if (type == "start") {
    event.type = MessageType::start;
    if (!only_members(root, {"version", "type", "seq", "height_start"}) ||
      !root.isMember("height_start") || !finite_number(root["height_start"])) {
      return reject("invalid_start_payload", now);
    }
    const double height = root["height_start"].asDouble();
    if (height < 0.2 || height > 1.0) {return reject("height_start_out_of_range", now);}
    event.height_start_m = height;
  } else if (type == "navigation_and_point") {
    event.type = MessageType::navigation_and_point;
    if (!only_members(root, {"version", "type", "seq", "plan_mode", "point"}) ||
      !root.isMember("plan_mode") || !root["plan_mode"].isInt() ||
      !root.isMember("point")) {return reject("invalid_navigation_and_point_payload", now);}
    Point3 point;
    if (!decode_point(root["point"], point)) {return reject("invalid_point", now);}
    if (origin_x_m_ && origin_y_m_ &&
      (point.x_m - *origin_x_m_ < config_.relative_x_min_m ||
      point.x_m - *origin_x_m_ > config_.relative_x_max_m ||
      point.y_m - *origin_y_m_ < config_.relative_y_min_m ||
      point.y_m - *origin_y_m_ > config_.relative_y_max_m ||
      point.z_m < config_.absolute_z_min_m || point.z_m > config_.absolute_z_max_m)) {
      return reject("point_out_of_safety_envelope", now);
    }
    event.plan_mode = root["plan_mode"].asInt();
    event.final_point = point;
  } else if (type == "navigation_nfz" || type == "navigation_plan") {
    const bool nfz = type == "navigation_nfz";
    event.type = nfz ? MessageType::navigation_nfz : MessageType::navigation_plan;
    const char * count_name = nfz ? "nfz_point_count" : "waypoint_count";
    const char * points_name = nfz ? "nfz_points" : "waypoints";
    if (!only_members(root, {"version", "type", "seq", count_name, points_name}) ||
      !root.isMember(count_name) || !root[count_name].isUInt() ||
      !root.isMember(points_name) || !root[points_name].isArray()) {
      return reject(nfz ? "invalid_navigation_nfz_payload" : "invalid_navigation_plan_payload", now);
    }
    if (root[count_name].asUInt64() != root[points_name].size()) {return reject("point_count_mismatch", now);}
    if (root[points_name].size() > 1024U) {return reject("too_many_points", now);}
    for (const auto & value : root[points_name]) {
      Point3 point;
      if (!decode_point(value, point)) {return reject("invalid_point_array", now);}
      if (!nfz && origin_x_m_ && origin_y_m_ &&
        (point.x_m - *origin_x_m_ < config_.relative_x_min_m ||
        point.x_m - *origin_x_m_ > config_.relative_x_max_m ||
        point.y_m - *origin_y_m_ < config_.relative_y_min_m ||
        point.y_m - *origin_y_m_ > config_.relative_y_max_m ||
        point.z_m < config_.absolute_z_min_m || point.z_m > config_.absolute_z_max_m)) {
        return reject("point_out_of_safety_envelope", now);
      }
      event.points.push_back(point);
    }
  } else if (type == "navigation_fly_plan_send_ok" || type == "ACK" ||
    type == "ok_fly_plan_succeed") {
    if (!only_members(root, {"version", "type", "seq"})) {return reject("unexpected_fields", now);}
    event.type = type == "navigation_fly_plan_send_ok" ? MessageType::navigation_fly_plan_send_ok :
      (type == "ACK" ? MessageType::ack : MessageType::ok_fly_plan_succeed);
  } else {
    return reject("unknown_type", now);
  }
  event.accepted = true;
  last_rejection_.clear();
  return event;
}

/// 每周期最多读取 64 个当前可用数据报并转换为协议事件，不发送逐包响应。
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
      if (errno != EAGAIN && errno != EWOULDBLOCK) {last_rejection_ = std::string("recv_error:") + std::strerror(errno);}
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

/// 为每条 UAV 输出消息分配从 1 开始的进程内唯一序号。
std::uint64_t GroundStationLink::next_tx_seq() {return ++tx_sequence_;}

/// 编码不带专用遥测字段的阶段业务消息。
std::string GroundStationLink::encode(const OutgoingMessage & message)
{
  std::ostringstream stream;
  stream << "{\"version\":1,\"type\":\"" << to_string(message.type)
         << "\",\"seq\":" << next_tx_seq();
  if (!message.detail.empty()) {stream << ",\"detail\":\"" << common::json_escape(message.detail) << "\"";}
  stream << "}";
  return stream.str();
}

/// 编码带明确 mavros_local_enu 坐标帧的绝对 XYZ 状态。
std::string GroundStationLink::encode_xyz_state(const common::Telemetry & telemetry)
{
  std::ostringstream stream;
  stream << "{\"version\":1,\"type\":\"xyz_state\",\"seq\":" << next_tx_seq()
         << ",\"frame\":\"mavros_local_enu\",\"x\":" << number(telemetry.local_x_m)
         << ",\"y\":" << number(telemetry.local_y_m) << ",\"z\":" << number(telemetry.local_z_m) << "}";
  return stream.str();
}

/// 编码电池存在性、电压和剩余比例，缺失数值使用 JSON null。
std::string GroundStationLink::encode_battery_state(const common::Telemetry & telemetry)
{
  std::ostringstream stream;
  stream << "{\"version\":1,\"type\":\"battery_state\",\"seq\":" << next_tx_seq()
         << ",\"present\":" << (telemetry.battery_present ? "true" : "false")
         << ",\"voltage\":" << number(telemetry.battery_voltage_v)
         << ",\"remaining\":" << number(telemetry.battery_fraction) << "}";
  return stream.str();
}

/// 使用 MSG_DONTWAIT 将一个完整 JSON 数据报发送到固定远端。
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

/// 编码并发送 Navigation 产生的单条业务输出。
bool GroundStationLink::send(const OutgoingMessage & message) {return send_json(encode(message));}

/// 达到配置周期时成对发送位置和电池状态，并更新最近发送时间。
bool GroundStationLink::send_status_if_due(const common::Telemetry & telemetry, double now)
{
  if (!std::isfinite(now) || now - last_status_at_ < config_.status_period_s) {return false;}
  last_status_at_ = now;
  const bool xyz_sent = send_json(encode_xyz_state(telemetry));
  const bool battery_sent = send_json(encode_battery_state(telemetry));
  return xyz_sent && battery_sent;
}

/// 锁存有限 local ENU XY 原点，为后续任务控制点应用相对包络。
void GroundStationLink::set_navigation_origin(double x_m, double y_m)
{
  if (!std::isfinite(x_m) || !std::isfinite(y_m)) {
    throw std::invalid_argument("navigation origin must be finite");
  }
  origin_x_m_ = x_m;
  origin_y_m_ = y_m;
}

}  // namespace mavros_xyz_position_offboard::communication
