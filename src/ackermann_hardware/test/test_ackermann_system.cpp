// Loads AckermannSystem through pluginlib (the way controller_manager does) and
// drives it with use_mock_hardware=true, so no CAN bus or PWM chip is needed.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"


namespace
{

constexpr double kPeriod = 0.01;
constexpr double kMaxSteer = 0.6981317;

std::string make_urdf(const std::string & extra_hardware_params = "")
{
  return R"(<?xml version="1.0"?>
<robot name="ackermann_test">
  <link name="base_link"/>
  <link name="rear_wheel"/>
  <link name="steering_link"/>
  <joint name="rear_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="rear_wheel"/><axis xyz="0 1 0"/>
  </joint>
  <joint name="steering_joint" type="revolute">
    <parent link="base_link"/><child link="steering_link"/><axis xyz="0 0 1"/>
    <limit lower="-0.7" upper="0.7" effort="1" velocity="6"/>
  </joint>
  <ros2_control name="AckermannBase" type="system">
    <hardware>
      <plugin>ackermann_hardware/AckermannSystem</plugin>
      <param name="use_mock_hardware">true</param>
      <param name="gear_ratio">0.5</param>
      <param name="max_wheel_velocity">15.0</param>
      <param name="steering_slew_rate">6.0</param>
)" + extra_hardware_params + R"(
    </hardware>
    <joint name="rear_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="steering_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
)";
}

class AckermannSystemTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("ackermann_system_test");
  }

  std::unique_ptr<hardware_interface::ResourceManager> load(const std::string & urdf)
  {
    return std::make_unique<hardware_interface::ResourceManager>(urdf);
  }

  static bool activate(hardware_interface::ResourceManager & rm)
  {
    rclcpp_lifecycle::State active(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
    return rm.set_component_state("AckermannBase", active) ==
           hardware_interface::return_type::OK;
  }

  void cycle(hardware_interface::ResourceManager & rm, int n = 1)
  {
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(kPeriod);
    for (int i = 0; i < n; ++i) {
      rm.write(node_->now(), period);
      rm.read(node_->now(), period);
    }
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(AckermannSystemTest, ExportsExpectedInterfaces)
{
  auto rm = load(make_urdf());
  EXPECT_TRUE(rm->state_interface_exists("rear_wheel_joint/position"));
  EXPECT_TRUE(rm->state_interface_exists("rear_wheel_joint/velocity"));
  EXPECT_TRUE(rm->state_interface_exists("steering_joint/position"));
  EXPECT_TRUE(rm->command_interface_exists("rear_wheel_joint/velocity"));
  EXPECT_TRUE(rm->command_interface_exists("steering_joint/position"));
  EXPECT_FALSE(rm->command_interface_exists("steering_joint/velocity"));
}

TEST_F(AckermannSystemTest, DrivesAndSteersInMockMode)
{
  auto rm = load(make_urdf());
  ASSERT_TRUE(activate(*rm));

  auto wheel_vel_cmd = rm->claim_command_interface("rear_wheel_joint/velocity");
  auto steer_cmd = rm->claim_command_interface("steering_joint/position");
  auto wheel_pos = rm->claim_state_interface("rear_wheel_joint/position");
  auto wheel_vel = rm->claim_state_interface("rear_wheel_joint/velocity");
  auto steer_pos = rm->claim_state_interface("steering_joint/position");

  (void)wheel_vel_cmd.set_value(4.0);
  (void)steer_cmd.set_value(0.3);

  cycle(*rm);
  EXPECT_NEAR(wheel_vel.get_value(), 4.0, 1e-9);
  EXPECT_NEAR(wheel_pos.get_value(), 4.0 * kPeriod, 1e-9);
  // The open-loop steering estimate is rate limited to 6 rad/s.
  EXPECT_NEAR(steer_pos.get_value(), 6.0 * kPeriod, 1e-9);

  cycle(*rm, 99);
  EXPECT_NEAR(wheel_pos.get_value(), 4.0 * kPeriod * 100, 1e-6);
  EXPECT_NEAR(steer_pos.get_value(), 0.3, 1e-9);
}

TEST_F(AckermannSystemTest, ClampsCommandsToLimits)
{
  auto rm = load(make_urdf());
  ASSERT_TRUE(activate(*rm));

  auto wheel_vel_cmd = rm->claim_command_interface("rear_wheel_joint/velocity");
  auto steer_cmd = rm->claim_command_interface("steering_joint/position");
  auto wheel_vel = rm->claim_state_interface("rear_wheel_joint/velocity");
  auto steer_pos = rm->claim_state_interface("steering_joint/position");

  (void)wheel_vel_cmd.set_value(100.0);
  (void)steer_cmd.set_value(-2.0);
  cycle(*rm, 50);
  EXPECT_NEAR(wheel_vel.get_value(), 15.0, 1e-9);
  EXPECT_NEAR(steer_pos.get_value(), -kMaxSteer, 1e-9);
}

TEST_F(AckermannSystemTest, NaNCommandStopsWheelAndHoldsSteering)
{
  auto rm = load(make_urdf());
  ASSERT_TRUE(activate(*rm));

  auto wheel_vel_cmd = rm->claim_command_interface("rear_wheel_joint/velocity");
  auto steer_cmd = rm->claim_command_interface("steering_joint/position");
  auto wheel_vel = rm->claim_state_interface("rear_wheel_joint/velocity");
  auto steer_pos = rm->claim_state_interface("steering_joint/position");

  (void)wheel_vel_cmd.set_value(2.0);
  (void)steer_cmd.set_value(0.2);
  cycle(*rm, 20);

  (void)wheel_vel_cmd.set_value(std::numeric_limits<double>::quiet_NaN());
  (void)steer_cmd.set_value(std::numeric_limits<double>::quiet_NaN());
  cycle(*rm, 5);
  EXPECT_DOUBLE_EQ(wheel_vel.get_value(), 0.0);
  EXPECT_NEAR(steer_pos.get_value(), 0.2, 1e-9);
}

TEST_F(AckermannSystemTest, RejectsInvalidParameters)
{
  // Humble's ResourceManager throws when a component fails on_init.
  EXPECT_THROW(
    load(make_urdf(R"(<param name="neutral_mode">hold</param>)")), std::runtime_error);
}

}  // namespace
