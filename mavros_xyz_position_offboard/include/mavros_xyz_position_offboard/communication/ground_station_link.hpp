#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "mavros_xyz_position_offboard/common/types.hpp"
#include "mavros_xyz_position_offboard/communication/protocol.hpp"

namespace mavros_xyz_position_offboard::communication
{

struct GroundStationConfig
{
  bool enabled{true};
  std::string bind_ip{"0.0.0.0"};
  int bind_port{5005};
  std::string remote_ip{"192.168.10.59"};
  int remote_port{5005};
  std::string whitelist_ip{"192.168.10.59"};
  int whitelist_port{5005};
  double status_period_s{0.5};
  double relative_x_min_m{0.0};
  double relative_x_max_m{4.0};
  double relative_y_min_m{0.0};
  double relative_y_max_m{5.0};
  double absolute_z_min_m{0.20};
  double absolute_z_max_m{1.00};

  /// 校验 UDP 地址、端口、周期和导航目标安全包络，失败时抛出异常。
  void validate() const;
};

/// Owns the non-blocking UDP endpoint and strict JSON V1 codec.
class GroundStationLink
{
public:
  /// 校验配置并按需创建、绑定非阻塞 UDP socket。
  explicit GroundStationLink(const GroundStationConfig & config);
  /// 关闭持有的 UDP socket。
  ~GroundStationLink();
  /// 禁止复制通信对象，避免重复拥有同一个 socket 文件描述符。
  GroundStationLink(const GroundStationLink &) = delete;
  /// 禁止复制赋值，保持 socket 和序号状态的唯一所有权。
  GroundStationLink & operator=(const GroundStationLink &) = delete;

  /// 严格解析、校验和去重一个 JSON V1 数据报，并返回值类型协议事件。
  ProtocolEvent decode_datagram(
    const std::string & payload, const std::string & source_ip, int source_port, double now);
  /// 清空当前可读 UDP 数据报，且不为输入包发送旧式逐包 ACK。
  std::vector<ProtocolEvent> poll(double now);
  /// 编码并向配置的固定远端发送一条业务消息。
  bool send(const OutgoingMessage & message);
  /// 到达配置周期时向固定远端发送 XYZ 和电池状态。
  bool send_status_if_due(const common::Telemetry & telemetry, double now);
  /// 保存初始化 local ENU XY 原点，供后续导航控制点安全包络校验。
  void set_navigation_origin(double x_m, double y_m);

  /// 为普通 UAV→GCS 业务消息生成带自增序号的 JSON V1 包络。
  std::string encode(const OutgoingMessage & message);
  /// 将 MAVROS local ENU 绝对位置编码为 xyz_state JSON V1 消息。
  std::string encode_xyz_state(const common::Telemetry & telemetry);
  /// 将电池存在性、电压和剩余比例编码为 battery_state JSON V1 消息。
  std::string encode_battery_state(const common::Telemetry & telemetry);

  /// 返回启动时锁存的通信配置。
  const GroundStationConfig & config() const {return config_;}
  /// 指示 UDP socket 是否已经成功创建并绑定。
  bool bound() const {return socket_fd_ >= 0;}
  /// 返回 socket 创建或绑定失败时记录的错误文本。
  const std::optional<std::string> & bind_error() const {return bind_error_;}
  /// 返回最近一次通信层拒绝或接收错误原因。
  const std::string & last_rejection() const {return last_rejection_;}

private:
  /// 构造拒绝事件并更新最近拒绝原因。
  ProtocolEvent reject(const std::string & reason, double now);
  /// 将已编码 JSON 数据报非阻塞发送到固定远端。
  bool send_json(const std::string & json);
  /// 生成下一条 UAV→GCS 消息使用的进程内递增序号。
  std::uint64_t next_tx_seq();

  GroundStationConfig config_;
  int socket_fd_{-1};
  std::optional<std::string> bind_error_{};
  std::unordered_set<std::uint64_t> received_sequences_{};
  std::uint64_t tx_sequence_{0};
  double last_status_at_{-1.0e100};
  std::string last_rejection_{};
  std::optional<double> origin_x_m_{};
  std::optional<double> origin_y_m_{};
};

}  // namespace mavros_xyz_position_offboard::communication
