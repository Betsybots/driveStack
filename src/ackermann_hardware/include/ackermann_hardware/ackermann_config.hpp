#ifndef ACKERMANN_HARDWARE__ACKERMANN_CONFIG_HPP_
#define ACKERMANN_HARDWARE__ACKERMANN_CONFIG_HPP_

// ROS-free configuration and kinematics helpers for the Ackermann hardware
// interface. Kept free of ROS and Phoenix includes so they can be unit tested
// on any machine.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace ackermann_hardware
{

// Signed 14-bit rotor position range reported by TalonFX (turns).
constexpr double ENCODER_MIN_TURNS = -16384.0;
constexpr double ENCODER_MAX_TURNS = 16383.999755859375;
constexpr double ENCODER_WRAP_RANGE_TURNS = ENCODER_MAX_TURNS - ENCODER_MIN_TURNS;

struct AckermannHardwareConfig
{
  // Skip CAN and PWM entirely; commands are mirrored back as states.
  bool use_mock_hardware = false;

  // Kraken X60 (TalonFX) on the rear axle.
  std::string canbus_name = "can0";
  int drive_motor_id = 0;
  // Wheel turns per motor turn; 0.5 = 2:1 reduction.
  double gear_ratio = 0.5;
  double max_wheel_velocity = 15.0;  // rad/s at the wheel
  double stator_current_limit = 40.0;
  double supply_current_limit = 20.0;
  std::string neutral_mode = "coast";
  bool drive_motor_inverted = true;
  // Slot 0 velocity gains (volts per rotor rps). kV ~= 12 V / 100 rps free speed for
  // a Kraken X60; tune kS/kP on the robot.
  double velocity_k_v = 0.12;
  double velocity_k_p = 0.1;
  double velocity_k_i = 0.0;
  double velocity_k_d = 0.0;
  double velocity_k_s = 0.0;
  double velocity_k_a = 0.0;
  // Motion-profile acceleration limit in rotor turns/s^2; 0 disables the limit.
  double velocity_acceleration = 0.0;

  // AGFRC A73BHLW servo on the Raspberry Pi hardware PWM (GPIO18 = pwmchip0/pwm0).
  int pwm_chip = 0;
  int pwm_channel = 0;
  long pwm_period_ns = 20000000;
  long servo_min_pulse_ns = 1000000;
  long servo_center_pulse_ns = 1500000;
  long servo_max_pulse_ns = 2000000;
  bool steering_inverted = false;
  double max_steer_angle = 0.6981317;  // rad
  double steering_slew_rate = 6.0;  // rad/s, used to estimate the open-loop servo angle
};

namespace detail
{

inline const std::string * find_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key)
{
  auto it = params.find(key);
  return it == params.end() ? nullptr : &it->second;
}

inline void read_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  std::string & out)
{
  if (const auto * v = find_param(params, key)) {
    out = *v;
  }
}

inline void read_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double & out)
{
  if (const auto * v = find_param(params, key)) {
    try {
      size_t used = 0;
      out = std::stod(*v, &used);
      if (used != v->size()) {
        throw std::invalid_argument("trailing characters");
      }
    } catch (const std::exception &) {
      throw std::invalid_argument("parameter '" + key + "' is not a number: '" + *v + "'");
    }
  }
}

inline void read_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  long & out)
{
  if (const auto * v = find_param(params, key)) {
    try {
      size_t used = 0;
      out = std::stol(*v, &used);
      if (used != v->size()) {
        throw std::invalid_argument("trailing characters");
      }
    } catch (const std::exception &) {
      throw std::invalid_argument("parameter '" + key + "' is not an integer: '" + *v + "'");
    }
  }
}

inline void read_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  int & out)
{
  long value = out;
  read_param(params, key, value);
  out = static_cast<int>(value);
}

inline void read_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  bool & out)
{
  if (const auto * v = find_param(params, key)) {
    std::string s = *v;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
    if (s == "true" || s == "1") {
      out = true;
    } else if (s == "false" || s == "0") {
      out = false;
    } else {
      throw std::invalid_argument("parameter '" + key + "' is not a bool: '" + *v + "'");
    }
  }
}

}  // namespace detail

// Builds the config from the <hardware><param> entries of the URDF, keeping the
// defaults for anything not given, and validates it. Throws std::invalid_argument.
inline AckermannHardwareConfig parse_config(
  const std::unordered_map<std::string, std::string> & params)
{
  using detail::read_param;
  AckermannHardwareConfig c;
  read_param(params, "use_mock_hardware", c.use_mock_hardware);
  read_param(params, "canbus_name", c.canbus_name);
  read_param(params, "drive_motor_id", c.drive_motor_id);
  read_param(params, "gear_ratio", c.gear_ratio);
  read_param(params, "max_wheel_velocity", c.max_wheel_velocity);
  read_param(params, "stator_current_limit", c.stator_current_limit);
  read_param(params, "supply_current_limit", c.supply_current_limit);
  read_param(params, "neutral_mode", c.neutral_mode);
  read_param(params, "drive_motor_inverted", c.drive_motor_inverted);
  read_param(params, "velocity_k_v", c.velocity_k_v);
  read_param(params, "velocity_k_p", c.velocity_k_p);
  read_param(params, "velocity_k_i", c.velocity_k_i);
  read_param(params, "velocity_k_d", c.velocity_k_d);
  read_param(params, "velocity_k_s", c.velocity_k_s);
  read_param(params, "velocity_k_a", c.velocity_k_a);
  read_param(params, "velocity_acceleration", c.velocity_acceleration);
  read_param(params, "pwm_chip", c.pwm_chip);
  read_param(params, "pwm_channel", c.pwm_channel);
  read_param(params, "pwm_period_ns", c.pwm_period_ns);
  read_param(params, "servo_min_pulse_ns", c.servo_min_pulse_ns);
  read_param(params, "servo_center_pulse_ns", c.servo_center_pulse_ns);
  read_param(params, "servo_max_pulse_ns", c.servo_max_pulse_ns);
  read_param(params, "steering_inverted", c.steering_inverted);
  read_param(params, "max_steer_angle", c.max_steer_angle);
  read_param(params, "steering_slew_rate", c.steering_slew_rate);

  if (c.gear_ratio <= 0.0 || c.max_wheel_velocity <= 0.0) {
    throw std::invalid_argument("gear_ratio and max_wheel_velocity must be positive");
  }
  if (c.stator_current_limit <= 0.0 || c.supply_current_limit <= 0.0 ||
    c.velocity_acceleration < 0.0)
  {
    throw std::invalid_argument("current limits must be positive and acceleration non-negative");
  }
  if (c.neutral_mode != "coast" && c.neutral_mode != "brake") {
    throw std::invalid_argument("neutral_mode must be 'coast' or 'brake'");
  }
  if (c.max_steer_angle <= 0.0 || c.max_steer_angle >= M_PI_2) {
    throw std::invalid_argument("max_steer_angle must be in (0, pi/2) radians");
  }
  if (c.steering_slew_rate <= 0.0) {
    throw std::invalid_argument("steering_slew_rate must be positive");
  }
  if (c.pwm_period_ns <= 0 || c.servo_min_pulse_ns <= 0 ||
    c.servo_min_pulse_ns >= c.servo_center_pulse_ns ||
    c.servo_center_pulse_ns >= c.servo_max_pulse_ns ||
    c.servo_max_pulse_ns > c.pwm_period_ns)
  {
    throw std::invalid_argument(
            "servo pulse widths must satisfy 0 < min < center < max <= pwm_period_ns");
  }
  return c;
}

// Linear map from steering angle to servo pulse, split at the centre pulse so an
// asymmetric servo trim still reaches both locks. Positive angle turns left
// (REP-103); flip steering_inverted if the servo turns the other way.
inline long steering_angle_to_pulse_ns(double angle, const AckermannHardwareConfig & c)
{
  const double direction = c.steering_inverted ? -1.0 : 1.0;
  const double normalized = std::clamp(direction * angle / c.max_steer_angle, -1.0, 1.0);
  const double span = normalized >= 0.0 ?
    static_cast<double>(c.servo_max_pulse_ns - c.servo_center_pulse_ns) :
    static_cast<double>(c.servo_center_pulse_ns - c.servo_min_pulse_ns);
  return std::lround(static_cast<double>(c.servo_center_pulse_ns) + normalized * span);
}

// Moves the open-loop steering estimate toward the target at no more than
// slew_rate rad/s.
inline double slew_toward(double current, double target, double slew_rate, double dt)
{
  const double max_step = slew_rate * std::max(dt, 0.0);
  return current + std::clamp(target - current, -max_step, max_step);
}

inline double wheel_rad_per_s_to_motor_tps(double wheel_rad_per_s, double gear_ratio)
{
  return wheel_rad_per_s / (2.0 * M_PI * gear_ratio);
}

inline double motor_turns_to_wheel_rad(double motor_turns, double gear_ratio)
{
  return motor_turns * 2.0 * M_PI * gear_ratio;
}

// Shortest signed step between two raw encoder readings, accounting for the
// TalonFX position wrapping at +/-16384 turns.
inline double unwrap_delta_turns(double current_turns, double previous_turns)
{
  double delta = current_turns - previous_turns;
  const double half_range = ENCODER_WRAP_RANGE_TURNS / 2.0;
  if (delta > half_range) {
    delta -= ENCODER_WRAP_RANGE_TURNS;
  } else if (delta < -half_range) {
    delta += ENCODER_WRAP_RANGE_TURNS;
  }
  return delta;
}

// Sanitises the controller's traction command: NaN (controller inactive) means stop.
inline double sanitize_wheel_velocity(double cmd, const AckermannHardwareConfig & c)
{
  if (!std::isfinite(cmd)) {
    return 0.0;
  }
  return std::clamp(cmd, -c.max_wheel_velocity, c.max_wheel_velocity);
}

// Sanitises the controller's steering command: NaN keeps the previous target.
inline double sanitize_steering(double cmd, double previous, const AckermannHardwareConfig & c)
{
  if (!std::isfinite(cmd)) {
    return previous;
  }
  return std::clamp(cmd, -c.max_steer_angle, c.max_steer_angle);
}

}  // namespace ackermann_hardware

#endif  // ACKERMANN_HARDWARE__ACKERMANN_CONFIG_HPP_
