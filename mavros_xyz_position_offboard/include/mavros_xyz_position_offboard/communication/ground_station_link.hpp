#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <vector>

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
  double event_retry_period_s{0.5};

  /// Validates endpoint and ordered-event retry configuration.
  void validate() const;
};

/// Owns the non-blocking UDP endpoint and the strict V2 JSON codec.
class GroundStationLink
{
public:
  explicit GroundStationLink(const GroundStationConfig & config);
  ~GroundStationLink();
  GroundStationLink(const GroundStationLink &) = delete;
  GroundStationLink & operator=(const GroundStationLink &) = delete;

  /// Strictly parses one GCS->UAV datagram and applies ACK queue progress.
  ProtocolEvent decode_datagram(
    const std::string & payload, const std::string & source_ip, int source_port, double now);
  /// Drains currently readable UDP datagrams without blocking.
  std::vector<ProtocolEvent> poll(double now);
  /// Enqueues an ordered ACK-required event and transmits the queue head when due.
  bool send(const OutgoingMessage & message, double now);
  /// Sends one fresh LCP sample immediately. xyzstatus is never queued or ACKed.
  bool send_xyzstatus(const XyzStatus & status);
  /// Resends the earliest unacknowledged event at the configured period.
  bool retry_events(double now);

  /// Encodes an event using the fixed V2 common structure.
  std::string encode(const OutgoingMessage & message) const;
  /// Encodes a complete xyzstatus document, including explicit null Z metadata when invalid.
  std::string encode_xyzstatus(const XyzStatus & status) const;

  const GroundStationConfig & config() const {return config_;}
  bool bound() const {return socket_fd_ >= 0;}
  const std::optional<std::string> & bind_error() const {return bind_error_;}
  const std::string & last_rejection() const {return last_rejection_;}
  std::size_t pending_event_count() const {return pending_events_.size();}
  std::optional<std::string> pending_event_json() const;

private:
  struct PendingEvent
  {
    std::string json{};
    double last_sent_at{-1.0e100};
  };

  ProtocolEvent reject(const std::string & reason, double now);
  bool send_json(const std::string & json);
  void acknowledge_earliest();

  GroundStationConfig config_;
  int socket_fd_{-1};
  std::optional<std::string> bind_error_{};
  std::deque<PendingEvent> pending_events_{};
  std::string last_rejection_{};
};

}  // namespace mavros_xyz_position_offboard::communication
