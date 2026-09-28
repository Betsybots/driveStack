#include "ackermann_hardware/ackermann_system.hpp"

#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ackermann_hardware/pwm_servo.hpp"

#if ACKERMANN_HARDWARE_WITH_PHOENIX6
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"
#endif

namespace ackermann_hardware
{

namespace
{

using hardware_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

rclcpp::Logger logger()
{
  return rclcpp::get_logger("AckermannSystem");
}

class PwmSteeringServo : public SteeringServo
{
public:
  PwmSteeringServo(int chip, int channel, long period_ns)
  : servo_(chip, channel, period_ns) {}

  void set_pulse_ns(long pulse_ns) override {servo_.set_pulse_ns(pulse_ns);}

private:
  SysfsPwmServo servo_;
};

#if ACKERMANN_HARDWARE_WITH_PHOENIX6
using namespace ctre::phoenix6;  // NOLINT(build/namespaces)

class TalonFxDriveMotor : public DriveMotor
{
public:
  explicit TalonFxDriveMotor(const AckermannHardwareConfig & c)
  : motor_(c.drive_motor_id, c.canbus_name)
  {
    configs::TalonFXConfiguration fx_cfg{};
    fx_cfg.MotorOutput.NeutralMode = c.neutral_mode == "brake" ?
      signals::NeutralModeValue::Brake :
      signals::NeutralModeValue::Coast;
    fx_cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    fx_cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{c.stator_current_limit};
    fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    fx_cfg.CurrentLimits.SupplyCurrentLimit = units::current::ampere_t{c.supply_current_limit};
    fx_cfg.MotorOutput.Inverted = c.drive_motor_inverted ?
      signals::InvertedValue::Clockwise_Positive :
      signals::InvertedValue::CounterClockwise_Positive;
    fx_cfg.Slot0.kV = c.velocity_k_v;
    fx_cfg.Slot0.kP = c.velocity_k_p;
    fx_cfg.Slot0.kI = c.velocity_k_i;
    fx_cfg.Slot0.kD = c.velocity_k_d;
    fx_cfg.Slot0.kS = c.velocity_k_s;
    fx_cfg.Slot0.kA = c.velocity_k_a;
    if (!motor_.GetConfigurator().Apply(fx_cfg).IsOK()) {
      throw std::runtime_error(
              "TalonFX " + std::to_string(c.drive_motor_id) + " on " + c.canbus_name +
              " did not accept its configuration; check CAN wiring and the motor ID");
    }

    velocity_request_.WithSlot(0).WithAcceleration(
      units::angular_acceleration::turns_per_second_squared_t{c.velocity_acceleration});
  }

  ~TalonFxDriveMotor() override
  {
    stop();
  }

  void set_velocity_tps(double motor_tps) override
  {
    velocity_request_.WithVelocity(units::angular_velocity::turns_per_second_t{motor_tps});
    motor_.SetControl(velocity_request_);
  }

  void stop() override
  {
    motor_.SetControl(neutral_request_);
  }

  void feed_enable() override
  {
    // The motor disables itself if this is not refreshed within 100 ms.
    ctre::phoenix::unmanaged::FeedEnable(100);
  }

  double position_turns() override {return motor_.GetPosition().GetValueAsDouble();}
  double velocity_tps() override {return motor_.GetVelocity().GetValueAsDouble();}

private:
  hardware::TalonFX motor_;
  controls::VelocityVoltage velocity_request_{units::angular_velocity::turns_per_second_t{0.0}};
  controls::NeutralOut neutral_request_{};
};
#endif

bool has_single_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return interfaces.size() == 1 && interfaces[0].name == name;
}

bool has_interfaces(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::vector<std::string> & names)
{
  if (interfaces.size() != names.size()) {
    return false;
  }
  for (size_t i = 0; i < names.size(); ++i) {
    if (interfaces[i].name != names[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace

CallbackReturn AckermannSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    config_ = parse_config(info_.hardware_parameters);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger(), "Invalid hardware parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

#if !ACKERMANN_HARDWARE_WITH_PHOENIX6
  if (!config_.use_mock_hardware) {
    RCLCPP_FATAL(
      logger(),
      "ackermann_hardware was built without Phoenix 6, so it can only run with "
      "use_mock_hardware=true. Install phoenix6 and rebuild to drive the real motor.");
    return CallbackReturn::ERROR;
  }
#endif

  // The traction joint is the one commanded in velocity, the steering joint the
  // one commanded in position; the joint names are free.
  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(
      logger(), "Expected exactly 2 joints (traction + steering), got %zu",
      info_.joints.size());
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    if (has_single_interface(joint.command_interfaces, HW_IF_VELOCITY)) {
      if (!has_interfaces(joint.state_interfaces, {HW_IF_POSITION, HW_IF_VELOCITY})) {
        RCLCPP_FATAL(
          logger(), "Traction joint '%s' must have state interfaces [position, velocity]",
          joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      traction_joint_ = joint.name;
    } else if (has_single_interface(joint.command_interfaces, HW_IF_POSITION)) {
      if (!has_single_interface(joint.state_interfaces, HW_IF_POSITION)) {
        RCLCPP_FATAL(
          logger(), "Steering joint '%s' must have a single position state interface",
          joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      steering_joint_ = joint.name;
    } else {
      RCLCPP_FATAL(
        logger(),
        "Joint '%s' must have exactly one command interface, velocity (traction) or "
        "position (steering)", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  if (traction_joint_.empty() || steering_joint_.empty()) {
    RCLCPP_FATAL(logger(), "Need one velocity-commanded and one position-commanded joint");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger(), "Traction joint '%s', steering joint '%s'%s", traction_joint_.c_str(),
    steering_joint_.c_str(), config_.use_mock_hardware ? " (mock hardware)" : "");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AckermannSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(traction_joint_, HW_IF_POSITION, &traction_position_state_);
  interfaces.emplace_back(traction_joint_, HW_IF_VELOCITY, &traction_velocity_state_);
  interfaces.emplace_back(steering_joint_, HW_IF_POSITION, &steering_position_state_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> AckermannSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(traction_joint_, HW_IF_VELOCITY, &traction_velocity_command_);
  interfaces.emplace_back(steering_joint_, HW_IF_POSITION, &steering_position_command_);
  return interfaces;
}

CallbackReturn AckermannSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    if (config_.use_mock_hardware) {
      drive_motor_ = std::make_unique<MockDriveMotor>();
      steering_servo_ = std::make_unique<MockSteeringServo>();
    } else {
#if ACKERMANN_HARDWARE_WITH_PHOENIX6
      drive_motor_ = std::make_unique<TalonFxDriveMotor>(config_);
#endif
      steering_servo_ = std::make_unique<PwmSteeringServo>(
        config_.pwm_chip, config_.pwm_channel, config_.pwm_period_ns);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Failed to open hardware: %s", e.what());
    release_actuators();
    return CallbackReturn::ERROR;
  }

  traction_velocity_command_ = 0.0;
  traction_velocity_state_ = 0.0;
  traction_position_state_ = 0.0;
  steering_position_command_ = 0.0;
  steering_position_state_ = 0.0;
  steering_target_ = 0.0;
  prev_motor_turns_ = drive_motor_->position_turns();
  motor_turns_unwrapped_ = 0.0;

  RCLCPP_INFO(logger(), "Configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Start from standstill with the wheels straight, whatever was left in the commands.
  traction_velocity_command_ = 0.0;
  steering_position_command_ = 0.0;
  steering_target_ = 0.0;
  try {
    drive_motor_->stop();
    center_steering();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Failed to activate hardware: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  try {
    drive_motor_->stop();
    center_steering();
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger(), "Error while stopping hardware: %s", e.what());
  }
  RCLCPP_INFO(logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  release_actuators();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  release_actuators();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AckermannSystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  drive_motor_->tick(dt);

  const double motor_turns = drive_motor_->position_turns();
  motor_turns_unwrapped_ += unwrap_delta_turns(motor_turns, prev_motor_turns_);
  prev_motor_turns_ = motor_turns;

  traction_position_state_ = motor_turns_to_wheel_rad(motor_turns_unwrapped_, config_.gear_ratio);
  traction_velocity_state_ =
    motor_turns_to_wheel_rad(drive_motor_->velocity_tps(), config_.gear_ratio);

  // The servo has no feedback, so report where it should be given its slew rate.
  steering_position_state_ = slew_toward(
    steering_position_state_, steering_target_, config_.steering_slew_rate, dt);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AckermannSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const double wheel_velocity = sanitize_wheel_velocity(traction_velocity_command_, config_);
  steering_target_ = sanitize_steering(steering_position_command_, steering_target_, config_);

  try {
    drive_motor_->feed_enable();
    drive_motor_->set_velocity_tps(
      wheel_rad_per_s_to_motor_tps(wheel_velocity, config_.gear_ratio));
    steering_servo_->set_pulse_ns(steering_angle_to_pulse_ns(steering_target_, config_));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      logger(), steady_clock_, 1000, "Write failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void AckermannSystem::center_steering()
{
  if (steering_servo_) {
    steering_servo_->set_pulse_ns(config_.servo_center_pulse_ns);
  }
}

void AckermannSystem::release_actuators()
{
  if (drive_motor_) {
    try {
      drive_motor_->stop();
    } catch (const std::exception &) {
      // Best effort: the motor also disables itself once feed_enable stops.
    }
  }
  drive_motor_.reset();
  // Unexports the PWM channel, which stops the pulse train.
  steering_servo_.reset();
}

}  // namespace ackermann_hardware

PLUGINLIB_EXPORT_CLASS(ackermann_hardware::AckermannSystem, hardware_interface::SystemInterface)
