#ifndef ACKERMANN_DRIVE_PWM_SERVO_HPP
#define ACKERMANN_DRIVE_PWM_SERVO_HPP

#include <chrono>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace ackermann_drive
{

// Steering servo driven through the Linux sysfs PWM interface (/sys/class/pwm).
// Requires the pwmchip to be exposed by the device tree and writable by this user.
class PwmServo
{
public:
    PwmServo(int chip, int channel, long period_ns)
    : base_("/sys/class/pwm/pwmchip" + std::to_string(chip)),
      path_(base_ + "/pwm" + std::to_string(channel)),
      channel_(channel)
    {
        if (!std::ifstream(path_ + "/period").good()) {
            write_or_throw(base_ + "/export", std::to_string(channel_));
            // The kernel creates pwmN/ asynchronously after export.
            for (int i = 0; i < 100 && !std::ifstream(path_ + "/period").good(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            if (!std::ifstream(path_ + "/period").good()) {
                throw std::runtime_error("PWM: " + path_ + " did not appear after export");
            }
        }

        write_or_throw(path_ + "/enable", "0");
        write_or_throw(path_ + "/duty_cycle", "0");
        write_or_throw(path_ + "/period", std::to_string(period_ns));
        exported_ = true;
    }

    ~PwmServo()
    {
        if (!exported_) {
            return;
        }
        try {
            write_or_throw(path_ + "/duty_cycle", "0");
            write_or_throw(path_ + "/enable", "0");
            write_or_throw(base_ + "/unexport", std::to_string(channel_));
        } catch (const std::exception &) {
            // Destructor must not throw; the channel is released on process exit anyway.
        }
    }

    PwmServo(const PwmServo &) = delete;
    PwmServo & operator=(const PwmServo &) = delete;

    void set_pulse_ns(long pulse_ns)
    {
        write_or_throw(path_ + "/duty_cycle", std::to_string(pulse_ns));
        if (!enabled_) {
            write_or_throw(path_ + "/enable", "1");
            enabled_ = true;
        }
    }

private:
    static void write_or_throw(const std::string & path, const std::string & value)
    {
        std::ofstream file(path);
        if (!file.is_open()) {
            throw std::runtime_error("PWM: cannot open " + path);
        }
        file << value;
        file.flush();
        if (file.fail()) {
            throw std::runtime_error("PWM: write failed on " + path);
        }
    }

    std::string base_;
    std::string path_;
    int channel_;
    bool exported_{false};
    bool enabled_{false};
};

}  // namespace ackermann_drive

#endif  // ACKERMANN_DRIVE_PWM_SERVO_HPP
