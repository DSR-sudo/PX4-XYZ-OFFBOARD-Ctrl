#pragma once

#include <fstream>
#include <optional>
#include <string>

namespace mavros_xyz_position_offboard::common
{

/// 对字符串进行 JSON 转义而不依赖第三方 JSON 库。
std::string json_escape(const std::string & value);
/// 生成用于日志记录的 UTC ISO-8601 时间戳。
std::string utc_timestamp();

class ArtifactLogger
{
public:
  /// 在指定目录创建 JSONL 或人类可读的飞行日志文件。
  ArtifactLogger(const std::string & directory, bool jsonl);
  /// 刷新并关闭仍处于打开状态的日志文件。
  ~ArtifactLogger();
  ArtifactLogger(const ArtifactLogger &) = delete;
  ArtifactLogger & operator=(const ArtifactLogger &) = delete;

  /// 追加一条状态记录并尽力立即落盘。
  void write(const std::string & record);
  /// 显式刷新并关闭日志文件。
  void close();
  /// 返回已成功打开的日志路径；打开失败时为空。
  const std::optional<std::string> & path() const {return path_;}

private:
  std::ofstream stream_;
  std::optional<std::string> path_;
  bool write_error_reported_{false};
};

}  // namespace mavros_xyz_position_offboard::common
