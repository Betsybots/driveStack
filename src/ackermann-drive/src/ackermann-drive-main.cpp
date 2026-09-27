#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#define ENABLE_MOTORS 1
#if ENABLE_MOTORS
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"
#endif

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "ackermann-drive/pwm_servo.hpp"

using namespace std::chrono_literals;
#if ENABLE_MOTORS
using namespace ctre::phoenix6;
#endif

constexpr double ENCODER_MIN_TURNS = -16384.0;
constexpr double ENCODER_MAX_TURNS = 16383.999755859375;
constexpr double ENCODER_WRAP_RANGE_TURNS = ENCODER_MAX_TURNS - ENCODER_MIN_TURNS;

// Below this speed the steering angle implied by a Twist is numerically meaningless.
constexpr double MIN_SPEED_FOR_STEERING_SOLVE = 0.02;

struct Pose2D
{
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

double wrap_angle_to_pi(double angle)
{
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
    return angle - M_PI;
}

class AckermannDrive : public rclcpp::Node
{
public:
    AckermannDrive() : Node("ackermann_drive")
    {
        canbus_name_ = this->declare_parameter<std::string>("canbus_name", "can0");
        // One motor drives a solid rear axle -- both wheels always spin at the same speed.
        drive_motor_id_ = this->declare_parameter<int>("drive_motor_id", 3);

        wheelbase_ = this->declare_parameter<double>("wheelbase", 0.31115);
        wheel_diameter_ = this->declare_parameter<double>("wheel_diameter", 0.13335);
        gear_ratio_ = this->declare_parameter<double>("gear_ratio", 0.5);
        max_steer_angle_ = this->declare_parameter<double>("max_steer_angle", 0.6981317);
        base_link_offset_from_rear_axle_ =
            this->declare_parameter<double>("base_link_offset_from_rear_axle", 0.0);

        stator_current_limit_ = this->declare_parameter<double>("stator_current_limit", 40.0);
        supply_current_limit_ = this->declare_parameter<double>("supply_current_limit", 20.0);
        neutral_mode_ = this->declare_parameter<std::string>("neutral_mode", "coast");
        // Matches the diff-drive setup's convention (both its motors were inverted: true).
        drive_motor_inverted_ = this->declare_parameter<bool>("drive_motor_inverted", true);
        velocity_k_v_ = this->declare_parameter<double>("velocity_k_v", 0.115);
        velocity_k_p_ = this->declare_parameter<double>("velocity_k_p", 0.15);
        velocity_k_i_ = this->declare_parameter<double>("velocity_k_i", 0.0);
        velocity_k_d_ = this->declare_parameter<double>("velocity_k_d", 0.01);
        velocity_k_s_ = this->declare_parameter<double>("velocity_k_s", 0.38);
        velocity_k_a_ = this->declare_parameter<double>("velocity_k_a", 0.2);
        velocity_acceleration_ = this->declare_parameter<double>("velocity_acceleration", 0.2);

        pwm_enabled_ = this->declare_parameter<bool>("pwm_enabled", true);
        pwm_chip_ = this->declare_parameter<int>("pwm_chip", 0);
        pwm_channel_ = this->declare_parameter<int>("pwm_channel", 0);
        pwm_period_ns_ = this->declare_parameter<int>("pwm_period_ns", 20000000);
        servo_min_pulse_ns_ = this->declare_parameter<int>("servo_min_pulse_ns", 1000000);
        servo_center_pulse_ns_ = this->declare_parameter<int>("servo_center_pulse_ns", 1500000);
        servo_max_pulse_ns_ = this->declare_parameter<int>("servo_max_pulse_ns", 2000000);
        steering_inverted_ = this->declare_parameter<bool>("steering_inverted", false);
        steering_slew_rate_ = this->declare_parameter<double>("steering_slew_rate", 6.0);

        max_linear_velocity_ = this->declare_parameter<double>("max_linear_velocity", 1.0);
        max_angular_velocity_ = this->declare_parameter<double>("max_angular_velocity", 2.0);
        cmd_vel_timeout_ms_ = this->declare_parameter<int>("cmd_vel_timeout_ms", 1000);
        update_period_ms_ = this->declare_parameter<int>("update_period_ms", 10);

        cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_smoothed");
        motor_speeds_topic_ =
            this->declare_parameter<std::string>("motor_speeds_topic", "/ackermann_drive/motor_speeds");
        wheel_odom_topic_ = this->declare_parameter<std::string>("wheel_odom_topic", "/wheel_odom");
        joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
        odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
        base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_footprint");
        // Both wheel joints are mechanically locked to the one motor, so both get the same position.
        left_joint_name_ = this->declare_parameter<std::string>("left_joint_name", "drivewhl_l_joint");
        right_joint_name_ = this->declare_parameter<std::string>("right_joint_name", "drivewhl_r_joint");
        steering_joint_name_ =
            this->declare_parameter<std::string>("steering_joint_name", "steering_joint");

        pose_covariance_xy_ = this->declare_parameter<double>("pose_covariance_xy", 1e-3);
        pose_covariance_z_roll_pitch_ =
            this->declare_parameter<double>("pose_covariance_z_roll_pitch", 1e-6);
        pose_covariance_yaw_ = this->declare_parameter<double>("pose_covariance_yaw", 5e-2);
        twist_covariance_vx_ = this->declare_parameter<double>("twist_covariance_vx", 1e-3);
        twist_covariance_vy_vz_roll_pitch_ =
            this->declare_parameter<double>("twist_covariance_vy_vz_roll_pitch", 1e-6);
        twist_covariance_yaw_rate_ = this->declare_parameter<double>("twist_covariance_yaw_rate", 2.5e-1);

        if (wheelbase_ <= 0.0 || wheel_diameter_ <= 0.0 || gear_ratio_ <= 0.0) {
            throw std::invalid_argument("wheelbase, wheel_diameter, and gear_ratio must be positive");
        }
        if (max_steer_angle_ <= 0.0 || max_steer_angle_ >= M_PI_2) {
            throw std::invalid_argument("max_steer_angle must be in (0, pi/2) radians");
        }
        if (cmd_vel_timeout_ms_ <= 0 || update_period_ms_ <= 0) {
            throw std::invalid_argument("cmd_vel_timeout_ms and update_period_ms must be positive");
        }
        if (max_linear_velocity_ <= 0.0 || max_angular_velocity_ <= 0.0 ||
            stator_current_limit_ <= 0.0 || supply_current_limit_ <= 0.0 ||
            velocity_acceleration_ < 0.0 || steering_slew_rate_ <= 0.0) {
            throw std::invalid_argument("velocity, current, and steering limits are invalid");
        }
        if (servo_min_pulse_ns_ >= servo_center_pulse_ns_ ||
            servo_center_pulse_ns_ >= servo_max_pulse_ns_ ||
            servo_max_pulse_ns_ > pwm_period_ns_) {
            throw std::invalid_argument(
                "servo pulse widths must satisfy min < center < max <= pwm_period_ns");
        }

        min_turning_radius_ = wheelbase_ / std::tan(max_steer_angle_);
        RCLCPP_INFO(this->get_logger(),
            "Ackermann geometry: wheelbase = %.4f m, max steer = %.1f deg, min turning radius = %.4f m",
            wheelbase_, max_steer_angle_ * 180.0 / M_PI, min_turning_radius_);

        if (pwm_enabled_) {
            steering_servo_ = std::make_unique<ackermann_drive::PwmServo>(
                pwm_chip_, pwm_channel_, pwm_period_ns_);
            steering_servo_->set_pulse_ns(servo_center_pulse_ns_);
        } else {
            RCLCPP_WARN(this->get_logger(), "pwm_enabled is false; steering servo will not be driven");
        }

#if ENABLE_MOTORS
        driveMotor = std::make_unique<hardware::TalonFX>(drive_motor_id_, canbus_name_);

        configs::TalonFXConfiguration fx_cfg{};
        if (neutral_mode_ == "coast") {
            fx_cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
        } else if (neutral_mode_ == "brake") {
            fx_cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
        } else {
            throw std::invalid_argument("neutral_mode must be 'coast' or 'brake'");
        }

        fx_cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        fx_cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{stator_current_limit_};
        fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        fx_cfg.CurrentLimits.SupplyCurrentLimit = units::current::ampere_t{supply_current_limit_};

        fx_cfg.MotorOutput.Inverted = drive_motor_inverted_
            ? signals::InvertedValue::Clockwise_Positive
            : signals::InvertedValue::CounterClockwise_Positive;
        driveMotor->GetConfigurator().Apply(fx_cfg);

        configs::Slot0Configs slot0Configs{};
        slot0Configs.kV = velocity_k_v_;
        slot0Configs.kP = velocity_k_p_;
        slot0Configs.kI = velocity_k_i_;
        slot0Configs.kD = velocity_k_d_;
        slot0Configs.kS = velocity_k_s_;
        slot0Configs.kA = velocity_k_a_;
        driveMotor->GetConfigurator().Apply(slot0Configs, 50_ms);

        drive_velocity.WithSlot(0).WithAcceleration(
            units::angular_acceleration::turns_per_second_squared_t{velocity_acceleration_});

        auto & drive_pos_signal = driveMotor->GetPosition().WaitForUpdate(100_ms);
        if (!drive_pos_signal.GetStatus().IsOK()) {
            RCLCPP_WARN(this->get_logger(),
                "Did not receive a fresh encoder position from CAN within timeout; "
                "odometry origin may be inaccurate at startup.");
        }
        prev_motor_pos_ = drive_pos_signal.GetValueAsDouble();
        motor_pos_unwrapped_ = prev_motor_pos_;
#endif

        last_cmd_time_ = this->now();
        last_update_time_ = this->now();

        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 5, std::bind(&AckermannDrive::cmd_vel_callback, this, std::placeholders::_1));

        motor_speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(motor_speeds_topic_, 10);
        motor_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(wheel_odom_topic_, 20);
        motor_pos_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(update_period_ms_), std::bind(&AckermannDrive::run_periodic, this));
    }

#if ENABLE_MOTORS
    std::unique_ptr<hardware::TalonFX> driveMotor;
    controls::VelocityVoltage drive_velocity{0_tps};
#endif

private:
    double unwrap_delta_turns(double current_turns, double previous_turns) const
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

    double wheel_linear_to_motor_tps(double wheel_linear_speed) const
    {
        return wheel_linear_speed / (M_PI * wheel_diameter_ * gear_ratio_);
    }

    void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr cmd)
    {
        if (!std::isfinite(cmd->linear.x) || !std::isfinite(cmd->angular.z)) {
            RCLCPP_ERROR(this->get_logger(), "Ignoring non-finite velocity command");
            return;
        }

        const double linear_x = std::clamp(cmd->linear.x, -max_linear_velocity_, max_linear_velocity_);
        const double angular_z = std::clamp(cmd->angular.z, -max_angular_velocity_, max_angular_velocity_);

        // delta = atan(L * wz / vx); undefined near standstill, so hold the previous angle there.
        if (std::abs(linear_x) >= MIN_SPEED_FOR_STEERING_SOLVE) {
            cmd_steer_angle_ = std::atan(wheelbase_ * angular_z / linear_x);
        }
        cmd_steer_angle_ = std::clamp(cmd_steer_angle_, -max_steer_angle_, max_steer_angle_);
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear_x=%.2f, angular_z=%.2f, cmd_steer_angle=%.2f",
            linear_x, angular_z, cmd_steer_angle_);

        cmd_body_speed_ = linear_x;
        last_cmd_time_ = this->now();
        cmd_vel_timeout_triggered_ = false;
    }

    void check_cmd_vel_watchdog()
    {
        const auto elapsed = this->now() - last_cmd_time_;
        if (elapsed > rclcpp::Duration(std::chrono::milliseconds(cmd_vel_timeout_ms_))) {
            if (!cmd_vel_timeout_triggered_) {
                RCLCPP_WARN(this->get_logger(),
                    "No command received on %s for %.2f s, stopping motors and centring steering",
                    cmd_vel_topic_.c_str(), elapsed.seconds());
                cmd_vel_timeout_triggered_ = true;
            }
            cmd_body_speed_ = 0.0;
            cmd_steer_angle_ = 0.0;
        }
    }

    void apply_steering(double dt)
    {
        // Open-loop servo: model the mechanical slew so odometry uses a realistic angle.
        const double max_step = steering_slew_rate_ * dt;
        const double error = cmd_steer_angle_ - steer_angle_estimate_;
        steer_angle_estimate_ += std::clamp(error, -max_step, max_step);

        if (!steering_servo_) {
            return;
        }

        const double direction = steering_inverted_ ? -1.0 : 1.0;
        const double normalized = direction * cmd_steer_angle_ / max_steer_angle_;
        const double span = normalized >= 0.0
            ? static_cast<double>(servo_max_pulse_ns_ - servo_center_pulse_ns_)
            : static_cast<double>(servo_center_pulse_ns_ - servo_min_pulse_ns_);

        const long pulse_ns = static_cast<long>(std::clamp(
            static_cast<double>(servo_center_pulse_ns_) + normalized * span,
            static_cast<double>(servo_min_pulse_ns_),
            static_cast<double>(servo_max_pulse_ns_)));
        RCLCPP_INFO(this->get_logger(),
            "Setting steering pulse to %ld ns for command angle %.2f rad, direction %.2f, span %.2f", pulse_ns, cmd_steer_angle_, direction, span);
        try {
            steering_servo_->set_pulse_ns(pulse_ns);
        } catch (const std::exception & e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Steering servo write failed: %s", e.what());
        }
    }

    void set_motor_speeds()
    {
#if ENABLE_MOTORS
        // Solid axle, single motor: no per-wheel speed split is possible.
        drive_velocity.WithVelocity(
            units::angular_velocity::turns_per_second_t{wheel_linear_to_motor_tps(cmd_body_speed_)});
        RCLCPP_INFO(this->get_logger(),
            "Setting drive motor velocity to %.2f turns per second", wheel_linear_to_motor_tps(cmd_body_speed_));
        driveMotor->SetControl(drive_velocity);
#endif
    }

    void calculate_and_publish_odometry()
    {
#if ENABLE_MOTORS
        const auto now = this->now();
        const double motor_turns = driveMotor->GetPosition().GetValueAsDouble();

        const double delta_turns = unwrap_delta_turns(motor_turns, prev_motor_pos_);
        prev_motor_pos_ = motor_turns;
        motor_pos_unwrapped_ += delta_turns;

        const double linear_dist = delta_turns * M_PI * wheel_diameter_ * gear_ratio_;

        // Bicycle model: yaw comes from steering angle, never from a wheel speed difference.
        const double delta_theta = linear_dist * std::tan(steer_angle_estimate_) / wheelbase_;
        rear_axle_pose_.x += linear_dist * std::cos(rear_axle_pose_.theta + delta_theta / 2.0);
        rear_axle_pose_.y += linear_dist * std::sin(rear_axle_pose_.theta + delta_theta / 2.0);
        rear_axle_pose_.theta = wrap_angle_to_pi(rear_axle_pose_.theta + delta_theta);

        const double body_speed =
            driveMotor->GetVelocity().GetValueAsDouble() * M_PI * wheel_diameter_ * gear_ratio_;
        const double yaw_rate = body_speed * std::tan(steer_angle_estimate_) / wheelbase_;

        const double base_x =
            rear_axle_pose_.x + base_link_offset_from_rear_axle_ * std::cos(rear_axle_pose_.theta);
        const double base_y =
            rear_axle_pose_.y + base_link_offset_from_rear_axle_ * std::sin(rear_axle_pose_.theta);
        const double half_theta = rear_axle_pose_.theta / 2.0;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = base_frame_id_;
        odom_msg.pose.pose.position.x = base_x;
        odom_msg.pose.pose.position.y = base_y;
        odom_msg.pose.pose.orientation.z = std::sin(half_theta);
        odom_msg.pose.pose.orientation.w = std::cos(half_theta);
        odom_msg.twist.twist.linear.x = body_speed;
        odom_msg.twist.twist.linear.y = yaw_rate * base_link_offset_from_rear_axle_;
        odom_msg.twist.twist.angular.z = yaw_rate;

        odom_msg.pose.covariance[0] = pose_covariance_xy_;
        odom_msg.pose.covariance[7] = pose_covariance_xy_;
        odom_msg.pose.covariance[14] = pose_covariance_z_roll_pitch_;
        odom_msg.pose.covariance[21] = pose_covariance_z_roll_pitch_;
        odom_msg.pose.covariance[28] = pose_covariance_z_roll_pitch_;
        odom_msg.pose.covariance[35] = pose_covariance_yaw_;
        odom_msg.twist.covariance[0] = twist_covariance_vx_;
        odom_msg.twist.covariance[7] = twist_covariance_vy_vz_roll_pitch_;
        odom_msg.twist.covariance[14] = twist_covariance_vy_vz_roll_pitch_;
        odom_msg.twist.covariance[21] = twist_covariance_vy_vz_roll_pitch_;
        odom_msg.twist.covariance[28] = twist_covariance_vy_vz_roll_pitch_;
        odom_msg.twist.covariance[35] = twist_covariance_yaw_rate_;
        motor_odom_publisher_->publish(odom_msg);

        geometry_msgs::msg::Twist speeds;
        speeds.linear.x = driveMotor->GetVelocity().GetValueAsDouble();
        speeds.angular.z = steer_angle_estimate_;
        motor_speed_publisher_->publish(speeds);

        // Solid axle: both wheel joints mirror the single motor's position.
        const double wheel_pos = motor_pos_unwrapped_ * 2.0 * M_PI * gear_ratio_;
        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = now;
        joint_msg.name = {left_joint_name_, right_joint_name_, steering_joint_name_};
        joint_msg.position = {wheel_pos, wheel_pos, steer_angle_estimate_};
        motor_pos_publisher_->publish(joint_msg);
#endif
    }

    void run_periodic()
    {
        const auto now = this->now();
        const double dt = std::max((now - last_update_time_).seconds(), 1e-6);
        last_update_time_ = now;

#if ENABLE_MOTORS
        ctre::phoenix::unmanaged::FeedEnable(100);
#endif
        check_cmd_vel_watchdog();
        apply_steering(dt);
        set_motor_speeds();
        calculate_and_publish_odometry();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_speed_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr motor_odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_pos_publisher_;

    std::unique_ptr<ackermann_drive::PwmServo> steering_servo_;

    std::string canbus_name_;
    int drive_motor_id_;
    double wheelbase_;
    double wheel_diameter_;
    double gear_ratio_;
    double max_steer_angle_;
    double min_turning_radius_{0.0};
    double base_link_offset_from_rear_axle_;
    double stator_current_limit_;
    double supply_current_limit_;
    std::string neutral_mode_;
    bool drive_motor_inverted_;
    double velocity_k_v_;
    double velocity_k_p_;
    double velocity_k_i_;
    double velocity_k_d_;
    double velocity_k_s_;
    double velocity_k_a_;
    double velocity_acceleration_;
    bool pwm_enabled_;
    int pwm_chip_;
    int pwm_channel_;
    int pwm_period_ns_;
    int servo_min_pulse_ns_;
    int servo_center_pulse_ns_;
    int servo_max_pulse_ns_;
    bool steering_inverted_;
    double steering_slew_rate_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    int cmd_vel_timeout_ms_;
    int update_period_ms_;
    std::string cmd_vel_topic_;
    std::string motor_speeds_topic_;
    std::string wheel_odom_topic_;
    std::string joint_states_topic_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string left_joint_name_;
    std::string right_joint_name_;
    std::string steering_joint_name_;
    double pose_covariance_xy_;
    double pose_covariance_z_roll_pitch_;
    double pose_covariance_yaw_;
    double twist_covariance_vx_;
    double twist_covariance_vy_vz_roll_pitch_;
    double twist_covariance_yaw_rate_;

    double cmd_body_speed_{0.0};
    double cmd_steer_angle_{0.0};
    double steer_angle_estimate_{0.0};
    bool cmd_vel_timeout_triggered_{false};
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_update_time_;

    double prev_motor_pos_{0.0};
    double motor_pos_unwrapped_{0.0};
    Pose2D rear_axle_pose_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannDrive>());
    rclcpp::shutdown();
    return 0;
}
