#ifndef ACKERMANN_HARDWARE__PWM_SERVO_HPP_
#define ACKERMANN_HARDWARE__PWM_SERVO_HPP_

#include <chrono>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace ackermann_hardware
{

// Hobby servo on a Linux sysfs PWM channel (/sys/class/pwm/pwmchipN/pwmM).
// On a Raspberry Pi, GPIO18 is pwmchip0/pwm0 once "dtoverlay=pwm" is enabled.
// The channel is exported on construction and released on destruction.
class SysfsPwmServo
{
public:
  SysfsPwmServo(int chip, int channel, long period_ns)
  : chip_dir_("/sys/class/pwm/pwmchip" + std::to_string(chip)),
    channel_dir_(chip_dir_ + "/pwm" + std::to_string(channel)),
    channel_(channel)
  {
    if (!exists(chip_dir_ + "/export")) {
      throw std::runtime_error(
              chip_dir_ + " not found; enable the PWM overlay (dtoverlay=pwm) and reboot");
    }
    if (!exists(channel_dir_ + "/period")) {
      write(chip_dir_ + "/export", std::to_string(channel_));
      // udev creates the channel directory asynchronously after the export.
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      while (!exists(channel_dir_ + "/period")) {
        if (std::chrono::steady_clock::now() > deadline) {
          throw std::runtime_error(channel_dir_ + " did not appear after export");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      exported_by_us_ = true;
    }

    // Duty cycle must never exceed the period, so zero it before changing the period.
    write(channel_dir_ + "/enable", "0");
    write(channel_dir_ + "/duty_cycle", "0");
    write(channel_dir_ + "/period", std::to_string(period_ns));
  }

  ~SysfsPwmServo()
  {
    try {
      write(channel_dir_ + "/enable", "0");
      if (exported_by_us_) {
        write(chip_dir_ + "/unexport", std::to_string(channel_));
      }
    } catch (const std::exception &) {
      // Nothing useful to do while tearing down.
    }
  }

  SysfsPwmServo(const SysfsPwmServo &) = delete;
  SysfsPwmServo & operator=(const SysfsPwmServo &) = delete;

  void set_pulse_ns(long pulse_ns)
  {
    if (pulse_ns == last_pulse_ns_) {
      return;
    }
    write(channel_dir_ + "/duty_cycle", std::to_string(pulse_ns));
    last_pulse_ns_ = pulse_ns;
    if (!enabled_) {
      write(channel_dir_ + "/enable", "1");
      enabled_ = true;
    }
  }

private:
  static bool exists(const std::string & path)
  {
    return std::ifstream(path).good();
  }

  static void write(const std::string & path, const std::string & value)
  {
    std::ofstream file(path);
    if (!file) {
      throw std::runtime_error("cannot open " + path + " (check permissions on /sys/class/pwm)");
    }
    file << value << std::flush;
    if (!file) {
      throw std::runtime_error("write of '" + value + "' to " + path + " failed");
    }
  }

  std::string chip_dir_;
  std::string channel_dir_;
  int channel_;
  bool exported_by_us_ = false;
  bool enabled_ = false;
  long last_pulse_ns_ = -1;
};

}  // namespace ackermann_hardware

#endif  // ACKERMANN_HARDWARE__PWM_SERVO_HPP_
