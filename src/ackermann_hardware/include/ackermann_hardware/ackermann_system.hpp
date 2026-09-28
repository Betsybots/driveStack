#ifndef ACKERMANN_HARDWARE__ACKERMANN_SYSTEM_HPP_
#define ACKERMANN_HARDWARE__ACKERMANN_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ackermann_hardware/ackermann_config.hpp"
#include "ackermann_hardware/actuators.hpp"

namespace ackermann_hardware
{

// ros2_control system for a rear-drive, front-steer Ackermann base:
//   - one traction joint: Kraken X60 (TalonFX, Phoenix 6) on a solid rear axle,
//     commanded in wheel rad/s, reporting wheel position and velocity;
//   - one steering joint: hobby servo on Raspberry Pi hardware PWM, commanded in
//     radians, reporting an open-loop estimate of the angle (the servo has no feedback).
// Pairs with bicycle_steering_controller from ros2_controllers.
class AckermannSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void release_actuators();
  void center_steering();

  AckermannHardwareConfig config_;
  std::string traction_joint_;
  std::string steering_joint_;

  std::unique_ptr<DriveMotor> drive_motor_;
  std::unique_ptr<SteeringServo> steering_servo_;

  // Commands written by the controller.
  double traction_velocity_command_ = 0.0;
  double steering_position_command_ = 0.0;

  // States read by the controller.
  double traction_position_state_ = 0.0;
  double traction_velocity_state_ = 0.0;
  double steering_position_state_ = 0.0;

  // Last valid steering target, held when the controller sends NaN.
  double steering_target_ = 0.0;
  double prev_motor_turns_ = 0.0;
  double motor_turns_unwrapped_ = 0.0;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace ackermann_hardware

#endif  // ACKERMANN_HARDWARE__ACKERMANN_SYSTEM_HPP_
