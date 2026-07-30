#include "mavros_xyz_position_offboard/gripper/pwm_gripper.hpp"

#include <lgpio.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace mavros_xyz_position_offboard::gripper
{
namespace
{

class LgpioPwmBackend final : public PwmGpioBackend
{
public:
  std::optional<int> find_rp1_gpiochip() override
  {
    namespace fs = std::filesystem;
    std::error_code error;
    const fs::path gpio_root{"/sys/class/gpio"};
    for (fs::directory_iterator gpio_it(gpio_root, error), end; !error && gpio_it != end;
      gpio_it.increment(error)) {
      const fs::path gpio_path = gpio_it->path();
      const std::string name = gpio_path.filename().string();
      if (name.rfind("gpiochip", 0) != 0) {continue;}

      std::ifstream label(gpio_path / "label");
      std::string label_value;
      std::getline(label, label_value);
      if (!label || label_value != "pinctrl-rp1") {continue;}

      const fs::path controller = fs::canonical(gpio_path / "device", error);
      if (error) {error.clear(); continue;}
      const fs::path char_root{"/sys/dev/char"};
      for (fs::directory_iterator char_it(char_root, error), char_end; !error && char_it != char_end;
        char_it.increment(error)) {
        const std::string device_name = char_it->path().filename().string();
        const auto separator = device_name.find(':');
        if (separator == std::string::npos || device_name.substr(0, separator) != "254") {continue;}
        const fs::path character_device = fs::canonical(char_it->path(), error);
        if (error) {error.clear(); continue;}
        if (character_device.parent_path() != controller) {continue;}
        try {
          return std::stoi(device_name.substr(separator + 1));
        } catch (const std::exception &) {
          return std::nullopt;
        }
      }
      if (error) {return std::nullopt;}
    }
    return std::nullopt;
  }

  int gpiochip_open(int gpiochip) override {return lgGpiochipOpen(gpiochip);}
  int gpiochip_close(int handle) override {return lgGpiochipClose(handle);}
  int gpio_claim_output(int handle, int bcm_gpio, int level) override
  {
    return lgGpioClaimOutput(handle, 0, bcm_gpio, level);
  }
  int gpio_free(int handle, int bcm_gpio) override {return lgGpioFree(handle, bcm_gpio);}
  int tx_pwm(int handle, int bcm_gpio, double frequency_hz, double duty_cycle) override
  {
    return lgTxPwm(handle, bcm_gpio, static_cast<float>(frequency_hz), static_cast<float>(duty_cycle), 0, 0);
  }
  std::string error_text(int status) const override {return lguErrorText(status);}
};

std::string error_message(const PwmGpioBackend & backend, const char * action, int status)
{
  return std::string(action) + " failed: " + backend.error_text(status) + " (" + std::to_string(status) + ")";
}

}  // namespace

std::shared_ptr<PwmGpioBackend> make_lgpio_pwm_backend()
{
  return std::make_shared<LgpioPwmBackend>();
}

void PwmGripperConfig::validate() const
{
  if (bcm_gpio < 0) {throw std::invalid_argument("BCM GPIO must be non-negative");}
  if (!std::isfinite(pwm_frequency_hz) || pwm_frequency_hz < 0.1 || pwm_frequency_hz > 10000.0) {
    throw std::invalid_argument("PWM frequency must be within 0.1..10000 Hz");
  }
  if (!std::isfinite(closed_duty_cycle) || !std::isfinite(open_duty_cycle) ||
    closed_duty_cycle <= 0.0 || closed_duty_cycle >= 100.0 ||
    open_duty_cycle <= 0.0 || open_duty_cycle >= 100.0) {
    throw std::invalid_argument("PWM duty cycles must be within 0..100 percent");
  }
  if (open_hold_ms < 0) {throw std::invalid_argument("open PWM hold time must be non-negative");}
}

PwmGripper::PwmGripper(PwmGripperConfig config, std::shared_ptr<PwmGpioBackend> backend)
: config_(std::move(config)), backend_(std::move(backend))
{
  config_.validate();
  if (!backend_) {throw std::invalid_argument("PWM GPIO backend is required");}
}

PwmGripper::~PwmGripper() {cleanup();}

void PwmGripper::cleanup() noexcept
{
  if (config_.enabled && handle_ >= 0) {
    if (gpio_claimed_) {
      backend_->tx_pwm(handle_, config_.bcm_gpio, 0.0, 0.0);
      backend_->gpio_free(handle_, config_.bcm_gpio);
    }
    backend_->gpiochip_close(handle_);
  }
  handle_ = -1;
  gpio_claimed_ = false;
  prepared_ = false;
}

void PwmGripper::fail(const std::string & reason)
{
  cleanup();
  fault_ = reason;
  phase_started_at_.reset();
  state_ = ReleaseState::failed;
}

bool PwmGripper::set_duty(double duty_cycle)
{
  const int status = backend_->tx_pwm(handle_, config_.bcm_gpio, config_.pwm_frequency_hz, duty_cycle);
  if (status < 0) {
    fail(error_message(*backend_, "setting SG90 PWM", status));
    return false;
  }
  return true;
}

bool PwmGripper::initialize()
{
  if (prepared_) {return true;}
  fault_.reset();
  if (!config_.enabled) {
    prepared_ = true;
    return true;
  }

  const auto gpiochip = backend_->find_rp1_gpiochip();
  if (!gpiochip) {
    fail("finding Raspberry Pi RP1 GPIO controller failed");
    return false;
  }
  handle_ = backend_->gpiochip_open(*gpiochip);
  if (handle_ < 0) {
    fail(error_message(*backend_, "opening RP1 gpiochip", handle_));
    return false;
  }
  const int claim_status = backend_->gpio_claim_output(handle_, config_.bcm_gpio, 0);
  if (claim_status < 0) {
    fail(error_message(*backend_, "claiming SG90 BCM GPIO", claim_status));
    return false;
  }
  gpio_claimed_ = true;
  if (!set_duty(config_.closed_duty_cycle)) {return false;}
  prepared_ = true;
  state_ = ReleaseState::idle;
  return true;
}

bool PwmGripper::begin_release(double now)
{
  if (!std::isfinite(now)) {throw std::invalid_argument("PWM action time must be finite");}
  if (state_ == ReleaseState::holding_release || state_ == ReleaseState::succeeded) {return true;}
  if (!prepared_ && !initialize()) {return false;}
  if (config_.enabled && !set_duty(config_.open_duty_cycle)) {return false;}
  phase_started_at_ = now;
  state_ = ReleaseState::holding_release;
  return true;
}

ReleaseState PwmGripper::update(double now)
{
  if (!std::isfinite(now)) {throw std::invalid_argument("PWM update time must be finite");}
  if (state_ != ReleaseState::holding_release || !phase_started_at_) {return state_;}
  if (now - *phase_started_at_ < static_cast<double>(config_.open_hold_ms) / 1000.0) {return state_;}
  if (config_.enabled && !set_duty(config_.closed_duty_cycle)) {return state_;}
  phase_started_at_.reset();
  state_ = ReleaseState::succeeded;
  return state_;
}

}  // mavros_xyz_position_offboard::gripper namespace
