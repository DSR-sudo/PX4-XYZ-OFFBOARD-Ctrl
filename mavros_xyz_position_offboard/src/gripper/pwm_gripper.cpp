#include "mavros_xyz_position_offboard/gripper/pwm_gripper.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace mavros_xyz_position_offboard::gripper
{
namespace
{
/// 将无符号整数转换为供 sysfs 写入的十进制文本。
std::string as_text(std::uint64_t value) {return std::to_string(value);}

/// 判断指定的可读文件是否包含预期文本。
bool readable_contains(const std::string & path, const std::string & expected)
{
  std::ifstream stream(path);
  if (!stream) {return false;}
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str().find(expected) != std::string::npos;
}
}  // 匿名命名空间

/// 校验 PWM 配置的路径、占空比关系、时序和引脚复用要求。
void PwmGripperConfig::validate() const
{
  if (channel < 0 || chip_path.empty()) {throw std::invalid_argument("PWM chip path and non-negative channel are required");}
  if (period_ns == 0 || idle_duty_ns == 0 || release_duty_ns == 0 || idle_duty_ns >= period_ns ||
    release_duty_ns >= period_ns) {
    throw std::invalid_argument("PWM period must exceed positive idle and release duties");
  }
  if (release_delay_ms < 0 || release_hold_ms < 0) {
    throw std::invalid_argument("PWM release delay and hold must be non-negative");
  }
  if (enabled && (pinmux_path.empty() || pinmux_expected.empty())) {
    throw std::invalid_argument("enabled PWM requires pinmux_path and pinmux_expected validation");
  }
}

/// 保存经过校验的 PWM 配置，延迟到释放时才访问硬件。
PwmGripper::PwmGripper(PwmGripperConfig config) : config_(std::move(config))
{
  config_.validate();
}

/// 拼接 sysfs 中当前 PWM 通道的目录路径。
std::string PwmGripper::channel_path() const
{
  return config_.chip_path + "/pwm" + std::to_string(config_.channel);
}

/// 将属性值写入 sysfs 文件，并确认写入流未发生错误。
bool PwmGripper::write_value(const std::string & path, const std::string & value) const
{
  std::ofstream stream(path);
  if (!stream) {return false;}
  stream << value;
  stream.flush();
  return static_cast<bool>(stream);
}

/// 验证配置的引脚复用状态是否满足 PWM 输出条件。
bool PwmGripper::verify_pinmux() const
{
  return readable_contains(config_.pinmux_path, config_.pinmux_expected);
}

/// 在需要时导出 PWM 通道，并确认其目录已经出现。
bool PwmGripper::ensure_channel()
{
  namespace fs = std::filesystem;
  const auto pwm_path = channel_path();
  std::error_code error;
  if (fs::exists(pwm_path, error)) {return true;}
  if (error) {return false;}
  if (!write_value(config_.chip_path + "/export", std::to_string(config_.channel))) {return false;}
  return fs::exists(pwm_path, error) && !error;
}

/// 将指定的纳秒占空比写入 PWM 通道。
bool PwmGripper::set_duty(std::uint64_t duty_ns)
{
  return write_value(channel_path() + "/duty_cycle", as_text(duty_ns));
}

/// 锁存失败原因，清除阶段计时并结束当前释放操作。
void PwmGripper::fail(const std::string & reason)
{
  fault_ = reason;
  phase_started_at_.reset();
  state_ = ReleaseState::failed;
}

/// 初始化 PWM 硬件并设置空闲占空比；禁用时仅标记为已准备。
bool PwmGripper::prepare()
{
  if (!config_.enabled) {prepared_ = true; return true;}
  namespace fs = std::filesystem;
  if (!verify_pinmux()) {fail("PWM pinmux validation failed"); return false;}
  std::error_code error;
  if (!fs::is_directory(config_.chip_path, error) || error) {
    fail("PWM chip path is not a readable directory");
    return false;
  }
  if (!ensure_channel()) {fail("PWM channel export or access failed"); return false;}
  const auto path = channel_path();
  if (!write_value(path + "/enable", "0") || !write_value(path + "/period", as_text(config_.period_ns)) ||
    !set_duty(config_.idle_duty_ns) || !write_value(path + "/enable", "1")) {
    fail("PWM sysfs period, duty, enable, or permission check failed");
    return false;
  }
  prepared_ = true;
  return true;
}

/// 启动释放状态机，并将重复成功调用视为幂等操作。
bool PwmGripper::begin_release(double now)
{
  if (!std::isfinite(now)) {throw std::invalid_argument("PWM action time must be finite");}
  if (state_ == ReleaseState::waiting_delay || state_ == ReleaseState::holding_release) {return true;}
  if (state_ == ReleaseState::succeeded) {return true;}
  fault_.reset();
  if (!prepared_ && !prepare()) {return false;}
  phase_started_at_ = now;
  state_ = ReleaseState::waiting_delay;
  return true;
}

/// 按配置时序切换释放/空闲占空比并返回当前状态。
ReleaseState PwmGripper::update(double now)
{
  if (!std::isfinite(now)) {throw std::invalid_argument("PWM update time must be finite");}
  if ((state_ != ReleaseState::waiting_delay && state_ != ReleaseState::holding_release) || !phase_started_at_) {
    return state_;
  }
  if (state_ == ReleaseState::waiting_delay &&
    now - *phase_started_at_ >= static_cast<double>(config_.release_delay_ms) / 1000.0) {
    if (config_.enabled && !set_duty(config_.release_duty_ns)) {
      fail("PWM release duty write failed");
      return state_;
    }
    phase_started_at_ = now;
    state_ = ReleaseState::holding_release;
  }
  if (state_ == ReleaseState::holding_release &&
    now - *phase_started_at_ >= static_cast<double>(config_.release_hold_ms) / 1000.0) {
    if (config_.enabled && !set_duty(config_.idle_duty_ns)) {
      fail("PWM idle duty restore failed");
      return state_;
    }
    phase_started_at_.reset();
    state_ = ReleaseState::succeeded;
  }
  return state_;
}

}  // mavros_xyz_position_offboard::gripper 命名空间
