#include "mavros_xyz_position_offboard/common/artifact_log.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <unistd.h>

namespace mavros_xyz_position_offboard::common
{

/// 将控制字符和引号转换成合法 JSON 字符串内容。
std::string json_escape(const std::string & value)
{
  std::ostringstream result;
  for (const unsigned char c : value) {
    switch (c) {
      case '\\': result << "\\\\"; break;
      case '"': result << "\\\""; break;
      case '\n': result << "\\n"; break;
      case '\r': result << "\\r"; break;
      case '\t': result << "\\t"; break;
      default:
        if (c < 0x20) {
          char buffer[7];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", c);
          result << buffer;
        } else {result << c;}
    }
  }
  return result.str();
}

/// 以毫秒精度生成 UTC ISO-8601 字符串。
std::string utc_timestamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::system_clock::to_time_t(now);
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
  std::tm tm{};
  gmtime_r(&seconds, &tm);
  char base[32];
  std::strftime(base, sizeof(base), "%Y-%m-%dT%H:%M:%S", &tm);
  char suffix[16];
  std::snprintf(suffix, sizeof(suffix), ".%03lldZ", static_cast<long long>(milliseconds));
  return std::string(base) + suffix;
}

/// 创建日志目录和带 UTC 时间/PID 的 artifact 文件。
ArtifactLogger::ArtifactLogger(const std::string & directory, bool jsonl)
{
  try {
    std::filesystem::path artifact_directory{directory};
    if (!artifact_directory.is_absolute()) {artifact_directory = std::filesystem::current_path() / artifact_directory;}
    std::filesystem::create_directories(artifact_directory);
    auto stamp = utc_timestamp();
    stamp.erase(std::remove(stamp.begin(), stamp.end(), ':'), stamp.end());
    stamp.erase(std::remove(stamp.begin(), stamp.end(), '-'), stamp.end());
    const auto filename = "mavros-xyz-flight-" + stamp + "-" + std::to_string(::getpid()) + (jsonl ? ".jsonl" : ".log");
    const auto file = artifact_directory / filename;
    stream_.open(file, std::ios::out | std::ios::app);
    if (!stream_) {throw std::runtime_error("open failed");}
    path_ = file.string();
  } catch (const std::exception & error) {
    std::cerr << "warning: unable to open artifact log in " << directory << ": " << error.what() << std::endl;
  }
}

/// 通过 close() 确保析构时不遗失缓冲数据。
ArtifactLogger::~ArtifactLogger() {close();}

/// 追加一条记录；首次写入故障只报告一次。
void ArtifactLogger::write(const std::string & record)
{
  if (!stream_) {return;}
  stream_ << record;
  if (record.empty() || record.back() != '\n') {stream_ << '\n';}
  stream_.flush();
  if (!stream_ && !write_error_reported_) {
    std::cerr << "warning: artifact log write failed" << std::endl;
    write_error_reported_ = true;
  }
}

/// 刷新并关闭打开的文件流，重复调用安全。
void ArtifactLogger::close()
{
  if (stream_.is_open()) {stream_.flush(); stream_.close();}
}

}  // namespace mavros_xyz_position_offboard::common
