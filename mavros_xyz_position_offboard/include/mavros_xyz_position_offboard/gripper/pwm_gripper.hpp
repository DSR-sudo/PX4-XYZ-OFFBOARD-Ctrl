#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace mavros_xyz_position_offboard::gripper
{

/// PWM 夹爪适配器的硬件路径、占空比和释放时序配置。
struct PwmGripperConfig
{
  bool enabled{false};
  std::string chip_path{"/sys/class/pwm/pwmchip0"};
  int channel{0};
  std::uint64_t period_ns{20000000};
  std::uint64_t idle_duty_ns{1500000};
  std::uint64_t release_duty_ns{2000000};
  int release_delay_ms{500};
  int release_hold_ms{500};
  std::string pinmux_path{};
  std::string pinmux_expected{};

  /// 校验 PWM 参数、时序以及启用硬件输出所需的引脚复用配置。
  void validate() const;
};

/// 一次释放操作的状态机阶段。
enum class ReleaseState
{
  idle,             ///< 空闲，尚未开始释放。
  waiting_delay,    ///< 已请求释放，正在等待配置的延时。
  holding_release,  ///< 正在保持释放占空比。
  succeeded,        ///< 已恢复空闲占空比，释放成功。
  failed            ///< 准备或 PWM 写入失败。
};

/// 基于 Raspberry Pi Linux PWM sysfs 的非阻塞夹爪适配器；禁用时为供 SITL 使用的定时空操作。
class PwmGripper
{
public:
  /// 保存并校验 PWM 配置，不会立即访问硬件。
  explicit PwmGripper(PwmGripperConfig config);

  /// 开始一次释放操作；成功后的重复调用幂等，失败后允许重试。
  bool begin_release(double now);
  /// 推进延时/保持阶段，并在报告成功前恢复空闲占空比。
  ReleaseState update(double now);
  /// 返回当前释放状态。
  ReleaseState state() const {return state_;}
  /// 返回最近一次失败原因；未失败时为空。
  const std::optional<std::string> & fault() const {return fault_;}
  /// 返回是否启用了实际 PWM 硬件输出。
  bool enabled() const {return config_.enabled;}

private:
  /// 校验硬件环境并初始化 PWM 通道为已启用的空闲占空比。
  bool prepare();
  /// 确保目标 PWM 通道已导出且可访问。
  bool ensure_channel();
  /// 验证引脚复用状态包含预期标识。
  bool verify_pinmux() const;
  /// 将字符串写入一个 sysfs 属性文件。
  bool write_value(const std::string & path, const std::string & value) const;
  /// 设置 PWM 占空比，单位为纳秒。
  bool set_duty(std::uint64_t duty_ns);
  /// 记录失败原因并将状态机切换到失败状态。
  void fail(const std::string & reason);
  /// 返回当前配置对应的 PWM 通道目录。
  std::string channel_path() const;

  PwmGripperConfig config_;
  ReleaseState state_{ReleaseState::idle};
  std::optional<double> phase_started_at_{};
  std::optional<std::string> fault_{};
  bool prepared_{false};
};

}  // mavros_xyz_position_offboard::gripper 命名空间
