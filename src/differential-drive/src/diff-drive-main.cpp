#include <chrono> // For using 10ms
#include "rclcpp/rclcpp.hpp"
#include <algorithm> // For std::clamp
#include <cmath> // For fmod/sin/cos
#include <memory>
#include <stdexcept>
#include <string>

#define ENABLE_MOTORS 1
#if ENABLE_MOTORS
#include "ctre/phoenix6/TalonFX.hpp"
#endif
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp" // for FeedEnable
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
#if ENABLE_MOTORS
using namespace ctre::phoenix6;
#endif

// M_PI is provided by <cmath> (glibc) at full double precision; no local override needed.

constexpr double ENCODER_MIN_TURNS = -16384.0;
constexpr double ENCODER_MAX_TURNS = 16383.999755859375;
constexpr double ENCODER_WRAP_RANGE_TURNS = ENCODER_MAX_TURNS - ENCODER_MIN_TURNS;
 
struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

// Wraps an angle (radians) into the (-pi, pi] range to prevent unbounded growth over long runtimes.
double wrap_angle_to_pi(double angle) {
    angle = fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
    return angle - M_PI;
}

class DifferentialDrive : public rclcpp::Node
{
public:

    DifferentialDrive() : Node("differential_drive")
    {
        canbus_name_ = this->declare_parameter<std::string>("canbus_name", "can0");
        left_motor_id_ = this->declare_parameter<int>("left_motor_id", 1);
        right_motor_id_ = this->declare_parameter<int>("right_motor_id", 2);
        wheel_distance_ = this->declare_parameter<double>("wheel_distance", 0.273812);
        wheel_diameter_ = this->declare_parameter<double>("wheel_diameter", 0.1524);
        gear_ratio_ = this->declare_parameter<double>("gear_ratio", 1.0);
        stator_current_limit_ = this->declare_parameter<double>("stator_current_limit", 40.0);
        supply_current_limit_ = this->declare_parameter<double>("supply_current_limit", 20.0);
        neutral_mode_ = this->declare_parameter<std::string>("neutral_mode", "coast");
        left_motor_inverted_ = this->declare_parameter<bool>("left_motor_inverted", false);
        right_motor_inverted_ = this->declare_parameter<bool>("right_motor_inverted", true);
        velocity_k_v_ = this->declare_parameter<double>("velocity_k_v", 0.12);
        velocity_k_p_ = this->declare_parameter<double>("velocity_k_p", 0.11);
        velocity_k_i_ = this->declare_parameter<double>("velocity_k_i", 0.52);
        velocity_k_d_ = this->declare_parameter<double>("velocity_k_d", 0.01);
        velocity_acceleration_ = this->declare_parameter<double>("velocity_acceleration", 0.2);
        max_linear_velocity_ = this->declare_parameter<double>("max_linear_velocity", 10.0);
        max_angular_velocity_ = this->declare_parameter<double>("max_angular_velocity", 20.0);
        invert_angular_velocity_ = this->declare_parameter<bool>("invert_angular_velocity", true);
        cmd_vel_timeout_ms_ = this->declare_parameter<int>("cmd_vel_timeout_ms", 1000);
        update_period_ms_ = this->declare_parameter<int>("update_period_ms", 10);
        cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        motor_speeds_topic_ = this->declare_parameter<std::string>("motor_speeds_topic", "/diff_drive/motor_speeds");
        wheel_odom_topic_ = this->declare_parameter<std::string>("wheel_odom_topic", "/wheel_odom");
        joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
        odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
        base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_footprint");
        left_joint_name_ = this->declare_parameter<std::string>("left_joint_name", "drivewhl_l_joint");
        right_joint_name_ = this->declare_parameter<std::string>("right_joint_name", "drivewhl_r_joint");
        pose_covariance_xy_ = this->declare_parameter<double>("pose_covariance_xy", 1e-3);
        pose_covariance_z_roll_pitch_ = this->declare_parameter<double>("pose_covariance_z_roll_pitch", 1e-6);
        pose_covariance_yaw_ = this->declare_parameter<double>("pose_covariance_yaw", 1e-3);
        twist_covariance_vx_ = this->declare_parameter<double>("twist_covariance_vx", 1e-3);
        twist_covariance_vy_vz_roll_pitch_ = this->declare_parameter<double>("twist_covariance_vy_vz_roll_pitch", 1e-6);
        twist_covariance_yaw_rate_ = this->declare_parameter<double>("twist_covariance_yaw_rate", 1e-3);

        if (wheel_distance_ <= 0.0 || wheel_diameter_ <= 0.0 || gear_ratio_ <= 0.0) {
            throw std::invalid_argument("wheel_distance, wheel_diameter, and gear_ratio must be positive");
        }
        if (cmd_vel_timeout_ms_ <= 0 || update_period_ms_ <= 0) {
            throw std::invalid_argument("cmd_vel_timeout_ms and update_period_ms must be positive");
        }
        if (max_linear_velocity_ <= 0.0 || max_angular_velocity_ <= 0.0 ||
            stator_current_limit_ <= 0.0 || supply_current_limit_ <= 0.0 ||
            velocity_acceleration_ < 0.0) {
            throw std::invalid_argument("velocity, current, and acceleration limits are invalid");
        }

#if ENABLE_MOTORS
        leftMotor = std::make_unique<hardware::TalonFX>(left_motor_id_, canbus_name_);
        rightMotor = std::make_unique<hardware::TalonFX>(right_motor_id_, canbus_name_);

        // Motor Config settings
        configs::TalonFXConfiguration fx_cfg{};
        if (neutral_mode_ == "coast") {
            fx_cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
        } else if (neutral_mode_ == "brake") {
            fx_cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
        } else {
            throw std::invalid_argument("neutral_mode must be 'coast' or 'brake'");
        }

        // enable stator current limit (applied identically to both motors below)
        fx_cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        fx_cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{stator_current_limit_};

        // enable supply current limit (applied identically to both motors below)
        fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        fx_cfg.CurrentLimits.SupplyCurrentLimit = units::current::ampere_t{supply_current_limit_};

        // the left motor is CCW+
        fx_cfg.MotorOutput.Inverted = left_motor_inverted_
            ? signals::InvertedValue::Clockwise_Positive
            : signals::InvertedValue::CounterClockwise_Positive;
        leftMotor->GetConfigurator().Apply(fx_cfg);
        
        // the right motor is CW+
        fx_cfg.MotorOutput.Inverted = right_motor_inverted_
            ? signals::InvertedValue::Clockwise_Positive
            : signals::InvertedValue::CounterClockwise_Positive;
        rightMotor->GetConfigurator().Apply(fx_cfg);

        RCLCPP_INFO(this->get_logger(),
            "Motor current limits (left & right): stator = %.1f A, supply = %.1f A",
            fx_cfg.CurrentLimits.StatorCurrentLimit.value(),
            fx_cfg.CurrentLimits.SupplyCurrentLimit.value());

        // robot init, set slot 0 gains
        configs::Slot0Configs slot0Configs{};
        slot0Configs.kV = velocity_k_v_;
        slot0Configs.kP = velocity_k_p_;
        slot0Configs.kI = velocity_k_i_;
        slot0Configs.kD = velocity_k_d_;
        fx_cfg.Slot0 = slot0Configs;
        
        leftMotor->GetConfigurator().Apply(slot0Configs, 50_ms);
        rightMotor->GetConfigurator().Apply(slot0Configs, 50_ms);
        
        left_velocity.WithSlot(0).WithAcceleration(units::angular_acceleration::turns_per_second_squared_t{velocity_acceleration_});
        right_velocity.WithSlot(0).WithAcceleration(units::angular_acceleration::turns_per_second_squared_t{velocity_acceleration_});
#endif
#if ENABLE_MOTORS
    // Block briefly for a real CAN-synced position before seeding the odometry integrator,
    // instead of trusting a possibly-stale/default cached value on the very first read.
    auto &left_pos_signal = leftMotor->GetPosition().WaitForUpdate(100_ms);
    auto &right_pos_signal = rightMotor->GetPosition().WaitForUpdate(100_ms);
    if (!left_pos_signal.GetStatus().IsOK() || !right_pos_signal.GetStatus().IsOK())
    {
        RCLCPP_WARN(this->get_logger(),
            "Did not receive a fresh encoder position from CAN within timeout; "
            "odometry origin may be inaccurate at startup.");
    }
    prev_left_motor_pos = left_pos_signal.GetValueAsDouble();
    prev_right_motor_pos = right_pos_signal.GetValueAsDouble();
    left_motor_pos_unwrapped = prev_left_motor_pos;
    right_motor_pos_unwrapped = prev_right_motor_pos;
#endif
        last_cmd_time_ = this->now();

        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 5, std::bind(&DifferentialDrive::cmd_vel_callback, this, std::placeholders::_1));

        motor_speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(motor_speeds_topic_, 10);
        motor_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(wheel_odom_topic_, 20);
        motor_pos_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);
        
        // Periodic Function to Drive
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(update_period_ms_), std::bind(&DifferentialDrive::run_periodic, this));
    }
    
#if ENABLE_MOTORS
    // devices
    std::unique_ptr<hardware::TalonFX> leftMotor;
    std::unique_ptr<hardware::TalonFX> rightMotor;

    // control requests
    controls::DutyCycleOut leftOut{0};
    controls::DutyCycleOut rightOut{0};
    controls::VelocityVoltage left_velocity{0_tps};
    controls::VelocityVoltage right_velocity{0_tps};
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

    void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr cmd)
    {
        if (!std::isfinite(cmd->linear.x) || !std::isfinite(cmd->angular.z)) {
            RCLCPP_ERROR(this->get_logger(), "Ignoring non-finite velocity command");
            return;
        }

        const double linear_x = std::clamp(cmd->linear.x, -max_linear_velocity_, max_linear_velocity_);
        const double angular_direction = invert_angular_velocity_ ? -1.0 : 1.0;
        const double angular_z = std::clamp(
            angular_direction * cmd->angular.z, -max_angular_velocity_, max_angular_velocity_);
        cmd_left_speed = (2.0 * linear_x - wheel_distance_ * angular_z) /
            (2.0 * M_PI * wheel_diameter_ * gear_ratio_);
        cmd_right_speed = (2.0 * linear_x + wheel_distance_ * angular_z) /
            (2.0 * M_PI * wheel_diameter_ * gear_ratio_);
        last_cmd_time_ = this->now();
        cmd_vel_timeout_triggered_ = false;
        new_cmd_received = true;
        RCLCPP_INFO(this->get_logger(), "ANgular vel = %f Linear vel = %f", angular_z, linear_x);
        RCLCPP_INFO(this->get_logger(), "Left_speed = %f right_speed = %f", cmd_left_speed, cmd_right_speed);
    }

    // Stops the motors if no velocity command has been received within the configured timeout.
    void check_cmd_vel_watchdog()
    {
        const auto elapsed = this->now() - last_cmd_time_;
        if (elapsed > rclcpp::Duration(std::chrono::milliseconds(cmd_vel_timeout_ms_)))
        {
            if (!cmd_vel_timeout_triggered_)
            {
                RCLCPP_WARN(this->get_logger(), "No /cmd_vel received for %.2f s, stopping motors", elapsed.seconds());
                cmd_vel_timeout_triggered_ = true;
            }
            cmd_left_speed = 0.0;
            cmd_right_speed = 0.0;
            new_cmd_received = true;
        }
    }
    
    void setMotorSpeeds()
    {
#if ENABLE_MOTORS
        //if(new_cmd_received)
        //{
            RCLCPP_INFO(this->get_logger(), "Setting Motor Speeds: Left = %f, Right = %f", cmd_left_speed, cmd_right_speed);
            // Set Motor Speeds
            left_velocity.WithVelocity(units::angular_velocity::turns_per_second_t{cmd_left_speed});
            right_velocity.WithVelocity(units::angular_velocity::turns_per_second_t{cmd_right_speed});
            leftMotor->SetControl(left_velocity);
            rightMotor->SetControl(right_velocity);
            new_cmd_received = false;
        //}
#endif
    }

    void calculate_and_publish_Odometry()
    {
        // Calculate Odometry based on wheel speeds and publish
        // This function can be implemented to calculate the robot's position and orientation
        // based on the wheel encoder readings and publish it as a nav_msgs::msg::Odometry message.
        const auto now = this->now();
        const double left_motor_turns = leftMotor->GetPosition().GetValueAsDouble();
        const double right_motor_turns = rightMotor->GetPosition().GetValueAsDouble();

        const double delta_left_motor_turns = unwrap_delta_turns(left_motor_turns, prev_left_motor_pos);
        const double delta_right_motor_turns = unwrap_delta_turns(right_motor_turns, prev_right_motor_pos);
        prev_left_motor_pos = left_motor_turns;
        prev_right_motor_pos = right_motor_turns;
        left_motor_pos_unwrapped += delta_left_motor_turns;
        right_motor_pos_unwrapped += delta_right_motor_turns;

        const double left_wheel_moved = delta_left_motor_turns * M_PI * wheel_diameter_ * gear_ratio_;
        const double right_wheel_moved = delta_right_motor_turns * M_PI * wheel_diameter_ * gear_ratio_;
        double linear_dist = (left_wheel_moved + right_wheel_moved) / 2.0;
        double angular_dist = (right_wheel_moved - left_wheel_moved) / wheel_distance_;
        robot_pose.x += linear_dist * cos(robot_pose.theta + angular_dist / 2.0);
        robot_pose.y += linear_dist * sin(robot_pose.theta + angular_dist / 2.0);
        robot_pose.theta = wrap_angle_to_pi(robot_pose.theta + angular_dist);
        const double half_theta = robot_pose.theta / 2.0;
        const double left_wheel_speed_meas = leftMotor->GetVelocity().GetValueAsDouble() * 2.0 * M_PI * gear_ratio_;
        const double right_wheel_speed_meas = rightMotor->GetVelocity().GetValueAsDouble() * 2.0 * M_PI * gear_ratio_;
 
        // Publish Odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = base_frame_id_;
        odom_msg.pose.pose.position.x = robot_pose.x;
        odom_msg.pose.pose.position.y = robot_pose.y;
        odom_msg.pose.pose.orientation.z = std::sin(half_theta);
        odom_msg.pose.pose.orientation.w = std::cos(half_theta);
        odom_msg.twist.twist.linear.x = static_cast<float>(
            (left_wheel_speed_meas + right_wheel_speed_meas) * wheel_diameter_ / 4.0);
        odom_msg.twist.twist.angular.z = static_cast<float>(
            (right_wheel_speed_meas - left_wheel_speed_meas) * wheel_diameter_ /
            (2.0 * wheel_distance_));

        // Diagonal covariance (off-diagonals remain 0, the message default).
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

        // Publish Motor Speeds and Joint States
        geometry_msgs::msg::Twist msg;
        msg.linear.x = static_cast<float>(leftMotor->GetVelocity().GetValueAsDouble());
        msg.linear.y = static_cast<float>(rightMotor->GetVelocity().GetValueAsDouble());
        motor_speed_publisher_->publish(msg);

        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = now;
        joint_msg.name = {left_joint_name_, right_joint_name_};
        joint_msg.position = {
            left_motor_pos_unwrapped * 2.0 * M_PI * gear_ratio_,
            right_motor_pos_unwrapped * 2.0 * M_PI * gear_ratio_
        };
        motor_pos_publisher_->publish(joint_msg);
    }

    void run_periodic()
    {
        ctre::phoenix::unmanaged::FeedEnable(100);
        check_cmd_vel_watchdog();
        setMotorSpeeds();
        calculate_and_publish_Odometry();  
    }

    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_speed_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr motor_odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_pos_publisher_;

    std::string canbus_name_;
    int left_motor_id_;
    int right_motor_id_;
    double wheel_distance_;
    double wheel_diameter_;
    double gear_ratio_;
    double stator_current_limit_;
    double supply_current_limit_;
    std::string neutral_mode_;
    bool left_motor_inverted_;
    bool right_motor_inverted_;
    double velocity_k_v_;
    double velocity_k_p_;
    double velocity_k_i_;
    double velocity_k_d_;
    double velocity_acceleration_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    bool invert_angular_velocity_;
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
    double pose_covariance_xy_;
    double pose_covariance_z_roll_pitch_;
    double pose_covariance_yaw_;
    double twist_covariance_vx_;
    double twist_covariance_vy_vz_roll_pitch_;
    double twist_covariance_yaw_rate_;
    
    double cmd_left_speed{0.0};
    double cmd_right_speed{0.0};

    double prev_left_motor_pos{0.0};
    double prev_right_motor_pos{0.0};
    double left_motor_pos_unwrapped{0.0};
    double right_motor_pos_unwrapped{0.0};

    Pose2D robot_pose;
    bool new_cmd_received = false;
    rclcpp::Time last_cmd_time_;
    bool cmd_vel_timeout_triggered_ = false;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DifferentialDrive>());
  rclcpp::shutdown();
  return 0;
}
