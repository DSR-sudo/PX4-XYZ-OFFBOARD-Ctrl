#pragma once

#include <memory>
#include <optional>
#include <string>

namespace mavros_xyz_position_offboard::gripper
{

/// SG90 GPIO 与 PWM 时序配置。
struct PwmGripperConfig
{
  bool enabled{false};
  int bcm_gpio{18};
  double pwm_frequency_hz{50.0};
  double closed_duty_cycle{4.0};
  double open_duty_cycle{7.0};
  int open_hold_ms{500};

  /// 校验 GPIO、PWM 占空比和开位保持时长。
  void validate() const;
};

/// 抽象 lgpio 调用，使定时状态机可在不访问真实 GPIO 的情况下测试。
class PwmGpioBackend
{
public:
  virtual ~PwmGpioBackend() = default;

  /// 返回 RP1 GPIO 控制器的动态 gpiochip 编号；找不到时返回空值。
  virtual std::optional<int> find_rp1_gpiochip() = 0;
  virtual int gpiochip_open(int gpiochip) = 0;
  virtual int gpiochip_close(int handle) = 0;
  virtual int gpio_claim_output(int handle, int bcm_gpio, int level) = 0;
  virtual int gpio_free(int handle, int bcm_gpio) = 0;
  virtual int tx_pwm(int handle, int bcm_gpio, double frequency_hz, double duty_cycle) = 0;
  virtual std::string error_text(int status) const = 0;
};

/// 创建通过 liblgpio 访问树莓派 GPIO 的生产后端。
std::shared_ptr<PwmGpioBackend> make_lgpio_pwm_backend();

/// 一次释放操作的状态机阶段。
enum class ReleaseState
{
  idle,             ///< 空闲，夹爪保持关闭位置。
  holding_release,  ///< 正在保持打开 PWM 占空比。
  succeeded,        ///< 已恢复关闭占空比，释放成功。
  failed            ///< 初始化或 PWM 写入失败。
};

/// 基于 Raspberry Pi 5 RP1 和 lgpio 的非阻塞 SG90 夹爪适配器。
class PwmGripper
{
public:
  explicit PwmGripper(
    PwmGripperConfig config, std::shared_ptr<PwmGpioBackend> backend = make_lgpio_pwm_backend());
  ~PwmGripper();
  PwmGripper(const PwmGripper &) = delete;
  PwmGripper & operator=(const PwmGripper &) = delete;

  /// 启用时定位 RP1、声明 BCM GPIO 并开始输出关闭位置 PWM；禁用时不访问硬件。
  bool initialize();
  /// 立即输出打开占空比；成功后的重复调用幂等，失败后允许重试。
  bool begin_release(double now);
  /// 在开位保持结束后恢复关闭占空比。
  ReleaseState update(double now);
  ReleaseState state() const {return state_;}
  const std::optional<std::string> & fault() const {return fault_;}
  bool enabled() const {return config_.enabled;}

private:
  bool set_duty(double duty_cycle);
  void fail(const std::string & reason);
  void cleanup() noexcept;

  PwmGripperConfig config_;
  std::shared_ptr<PwmGpioBackend> backend_;
  ReleaseState state_{ReleaseState::idle};
  std::optional<double> phase_started_at_{};
  std::optional<std::string> fault_{};
  int handle_{-1};
  bool gpio_claimed_{false};
  bool prepared_{false};
};

}  // mavros_xyz_position_offboard::gripper namespace
