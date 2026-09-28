#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "ackermann_hardware/ackermann_config.hpp"

using ackermann_hardware::AckermannHardwareConfig;
using ackermann_hardware::parse_config;

namespace
{
using Params = std::unordered_map<std::string, std::string>;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

TEST(ParseConfig, EmptyParamsGiveDefaults)
{
  const auto c = parse_config({});
  EXPECT_FALSE(c.use_mock_hardware);
  EXPECT_EQ(c.canbus_name, "can0");
  EXPECT_EQ(c.drive_motor_id, 0);
  EXPECT_DOUBLE_EQ(c.gear_ratio, 0.5);
  EXPECT_EQ(c.servo_center_pulse_ns, 1500000);
  EXPECT_EQ(c.neutral_mode, "coast");
}

TEST(ParseConfig, ReadsOverrides)
{
  const auto c = parse_config(
    Params{
      {"use_mock_hardware", "True"},
      {"canbus_name", "can1"},
      {"drive_motor_id", "7"},
      {"gear_ratio", "0.25"},
      {"neutral_mode", "brake"},
      {"steering_inverted", "1"},
      {"servo_min_pulse_ns", "900000"},
      {"max_steer_angle", "0.5"},
    });
  EXPECT_TRUE(c.use_mock_hardware);
  EXPECT_EQ(c.canbus_name, "can1");
  EXPECT_EQ(c.drive_motor_id, 7);
  EXPECT_DOUBLE_EQ(c.gear_ratio, 0.25);
  EXPECT_EQ(c.neutral_mode, "brake");
  EXPECT_TRUE(c.steering_inverted);
  EXPECT_EQ(c.servo_min_pulse_ns, 900000);
  EXPECT_DOUBLE_EQ(c.max_steer_angle, 0.5);
}

TEST(ParseConfig, RejectsMalformedValues)
{
  EXPECT_THROW(parse_config(Params{{"gear_ratio", "half"}}), std::invalid_argument);
  EXPECT_THROW(parse_config(Params{{"gear_ratio", "0.5x"}}), std::invalid_argument);
  EXPECT_THROW(parse_config(Params{{"drive_motor_id", "1.5"}}), std::invalid_argument);
  EXPECT_THROW(parse_config(Params{{"use_mock_hardware", "yes"}}), std::invalid_argument);
}

TEST(ParseConfig, RejectsInvalidRanges)
{
  EXPECT_THROW(parse_config(Params{{"gear_ratio", "0"}}), std::invalid_argument);
  EXPECT_THROW(parse_config(Params{{"max_wheel_velocity", "-1"}}), std::invalid_argument);
  EXPECT_THROW(parse_config(Params{{"neutral_mode", "hold"}}), std::invalid_argument);
  EXPECT_THROW(parse_config(Params{{"max_steer_angle", "1.6"}}), std::invalid_argument);
  EXPECT_THROW(parse_config(Params{{"steering_slew_rate", "0"}}), std::invalid_argument);
  EXPECT_THROW(
    parse_config(Params{{"servo_min_pulse_ns", "1600000"}}), std::invalid_argument);
  EXPECT_THROW(
    parse_config(Params{{"servo_max_pulse_ns", "30000000"}}), std::invalid_argument);
}

TEST(SteeringPulse, MapsCenterAndLocks)
{
  const AckermannHardwareConfig c;
  EXPECT_EQ(ackermann_hardware::steering_angle_to_pulse_ns(0.0, c), 1500000);
  EXPECT_EQ(ackermann_hardware::steering_angle_to_pulse_ns(c.max_steer_angle, c), 2000000);
  EXPECT_EQ(ackermann_hardware::steering_angle_to_pulse_ns(-c.max_steer_angle, c), 1000000);
  EXPECT_EQ(
    ackermann_hardware::steering_angle_to_pulse_ns(c.max_steer_angle / 2.0, c), 1750000);
}

TEST(SteeringPulse, ClampsBeyondLock)
{
  const AckermannHardwareConfig c;
  EXPECT_EQ(ackermann_hardware::steering_angle_to_pulse_ns(10.0, c), 2000000);
  EXPECT_EQ(ackermann_hardware::steering_angle_to_pulse_ns(-10.0, c), 1000000);
}

TEST(SteeringPulse, InvertedSwapsSides)
{
  AckermannHardwareConfig c;
  c.steering_inverted = true;
  EXPECT_EQ(ackermann_hardware::steering_angle_to_pulse_ns(c.max_steer_angle, c), 1000000);
  EXPECT_EQ(ackermann_hardware::steering_angle_to_pulse_ns(-c.max_steer_angle, c), 2000000);
}

TEST(SteeringPulse, AsymmetricTrimReachesBothLocks)
{
  AckermannHardwareConfig c;
  c.servo_min_pulse_ns = 1100000;
  c.servo_center_pulse_ns = 1450000;
  c.servo_max_pulse_ns = 1950000;
  EXPECT_EQ(ackermann_hardware::steering_angle_to_pulse_ns(0.0, c), 1450000);
  EXPECT_EQ(ackermann_hardware::steering_angle_to_pulse_ns(c.max_steer_angle, c), 1950000);
  EXPECT_EQ(ackermann_hardware::steering_angle_to_pulse_ns(-c.max_steer_angle, c), 1100000);
}

TEST(SlewToward, LimitsStepAndReachesTarget)
{
  EXPECT_DOUBLE_EQ(ackermann_hardware::slew_toward(0.0, 1.0, 6.0, 0.01), 0.06);
  EXPECT_DOUBLE_EQ(ackermann_hardware::slew_toward(0.0, -1.0, 6.0, 0.01), -0.06);
  EXPECT_DOUBLE_EQ(ackermann_hardware::slew_toward(0.98, 1.0, 6.0, 0.01), 1.0);
  EXPECT_DOUBLE_EQ(ackermann_hardware::slew_toward(0.5, 1.0, 6.0, -0.01), 0.5);
}

TEST(Conversions, WheelAndMotorRoundTrip)
{
  // 2:1 reduction: one wheel turn needs two motor turns.
  EXPECT_DOUBLE_EQ(ackermann_hardware::wheel_rad_per_s_to_motor_tps(2.0 * M_PI, 0.5), 2.0);
  EXPECT_DOUBLE_EQ(ackermann_hardware::motor_turns_to_wheel_rad(2.0, 0.5), 2.0 * M_PI);
  const double wheel = 3.7;
  EXPECT_NEAR(
    ackermann_hardware::motor_turns_to_wheel_rad(
      ackermann_hardware::wheel_rad_per_s_to_motor_tps(wheel, 0.5), 0.5),
    wheel, 1e-12);
}

TEST(UnwrapDelta, HandlesEncoderWrap)
{
  EXPECT_DOUBLE_EQ(ackermann_hardware::unwrap_delta_turns(10.5, 10.0), 0.5);
  EXPECT_NEAR(ackermann_hardware::unwrap_delta_turns(-16383.75, 16383.75), 0.5, 1e-3);
  EXPECT_NEAR(ackermann_hardware::unwrap_delta_turns(16383.75, -16383.75), -0.5, 1e-3);
}

TEST(Sanitize, NaNStopsDriveAndHoldsSteering)
{
  const AckermannHardwareConfig c;
  EXPECT_DOUBLE_EQ(ackermann_hardware::sanitize_wheel_velocity(kNaN, c), 0.0);
  EXPECT_DOUBLE_EQ(ackermann_hardware::sanitize_wheel_velocity(100.0, c), c.max_wheel_velocity);
  EXPECT_DOUBLE_EQ(ackermann_hardware::sanitize_wheel_velocity(-3.0, c), -3.0);
  EXPECT_DOUBLE_EQ(ackermann_hardware::sanitize_steering(kNaN, 0.2, c), 0.2);
  EXPECT_DOUBLE_EQ(ackermann_hardware::sanitize_steering(5.0, 0.0, c), c.max_steer_angle);
}
