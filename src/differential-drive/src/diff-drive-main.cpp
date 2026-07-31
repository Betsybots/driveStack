#include <chrono> // For using 10ms
#include "rclcpp/rclcpp.hpp"
#include <algorithm> // For std::clamp
#include <cmath> // For fmod/sin/cos

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

#define WHEEL_DISTANCE (double)0.339725
#define WHEEL_DIAMETER (double)0.1524 //0.1905
#define GEAR_RATIO (double)0.2 //0.3
// M_PI is provided by <cmath> (glibc) at full double precision; no local override needed.

constexpr char const *CANBUS_NAME = "can0";
constexpr double ENCODER_MIN_TURNS = -16384.0;
constexpr double ENCODER_MAX_TURNS = 16383.999755859375;
constexpr double ENCODER_WRAP_RANGE_TURNS = ENCODER_MAX_TURNS - ENCODER_MIN_TURNS;

// If no /cmd_vel message is received within this window, motors are commanded to stop.
constexpr auto CMD_VEL_TIMEOUT = 300ms;

// Maximum commanded robot linear velocity, in m/s. TUNE to your robot's safe operating limit.
constexpr double MAX_LINEAR_VELOCITY_MPS = 1.9;

// Maximum commanded robot angular velocity, in rad/s. TUNE to your robot's safe operating limit.
constexpr double MAX_ANGULAR_VELOCITY_RADPS = 8.9;

// Diagonal covariance values for the published wheel odometry (row-major 6x6: x,y,z,roll,pitch,yaw).
// z/roll/pitch are physically constrained to 0 on a planar robot, so they get tight covariance.
// x/y/yaw and vx/vyaw reflect typical wheel-encoder odometry uncertainty; vy is unobserved (no lateral slip modeled).
constexpr double POSE_COV_XY = 1e-3;
constexpr double POSE_COV_Z_ROLL_PITCH = 1e-6;
constexpr double POSE_COV_YAW = 1e-3;
constexpr double TWIST_COV_VX = 1e-3;
constexpr double TWIST_COV_VY_VZ_ROLLRATE_PITCHRATE = 1e-6;
constexpr double TWIST_COV_YAW_RATE = 1e-3;
 
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
#if ENABLE_MOTORS
        // Motor Config settings
        configs::TalonFXConfiguration fx_cfg{};
        fx_cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;

        // the left motor is CCW+
        fx_cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
        leftMotor.GetConfigurator().Apply(fx_cfg);
        
        // the right motor is CW+
        fx_cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
        rightMotor.GetConfigurator().Apply(fx_cfg);
        
        // robot init, set slot 0 gains
        configs::Slot0Configs slot0Configs{};
        slot0Configs.kV = 0.12;
        slot0Configs.kP = 0.11;
        slot0Configs.kI = 0.52;
        slot0Configs.kD = 0.01;
        fx_cfg.Slot0 = slot0Configs;
        
        leftMotor.GetConfigurator().Apply(slot0Configs, 50_ms);
        rightMotor.GetConfigurator().Apply(slot0Configs, 50_ms);
        
        left_velocity.WithSlot(0).WithAcceleration(0.2_tr_per_s_sq);
        right_velocity.WithSlot(0).WithAcceleration(0.2_tr_per_s_sq);
#endif
#if ENABLE_MOTORS
    // Block briefly for a real CAN-synced position before seeding the odometry integrator,
    // instead of trusting a possibly-stale/default cached value on the very first read.
    auto &left_pos_signal = leftMotor.GetPosition().WaitForUpdate(100_ms);
    auto &right_pos_signal = rightMotor.GetPosition().WaitForUpdate(100_ms);
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
        "/cmd_vel", 5, std::bind(&DifferentialDrive::cmd_vel_callback, this, std::placeholders::_1));

        motor_speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive/motor_speeds", 10);
        motor_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 20);
        motor_pos_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        
        // Periodic Function to Drive
        timer_ = this->create_wall_timer(10ms, std::bind(&DifferentialDrive::run_periodic, this));
    }
    
#if ENABLE_MOTORS
    // devices
    hardware::TalonFX leftMotor{1, CANBUS_NAME};
    hardware::TalonFX rightMotor{2, CANBUS_NAME};

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
        const double linear_x = std::clamp(cmd->linear.x, -MAX_LINEAR_VELOCITY_MPS, MAX_LINEAR_VELOCITY_MPS);
        const double angular_z = std::clamp(cmd->angular.z, -MAX_ANGULAR_VELOCITY_RADPS, MAX_ANGULAR_VELOCITY_RADPS);
        cmd_left_speed = (2.0 * linear_x - WHEEL_DISTANCE * angular_z)/(double)(2.0 * M_PI * WHEEL_DIAMETER * GEAR_RATIO);
        cmd_right_speed = (2.0 * linear_x + WHEEL_DISTANCE * angular_z)/(double)(2.0 * M_PI * WHEEL_DIAMETER * GEAR_RATIO);
        last_cmd_time_ = this->now();
        cmd_vel_timeout_triggered_ = false;
        new_cmd_received = true;
        RCLCPP_INFO(this->get_logger(), "Left_speed = %f right_speed = %f", cmd_left_speed, cmd_right_speed);
    }

    // Stops the motors if no /cmd_vel message has been received within CMD_VEL_TIMEOUT.
    void check_cmd_vel_watchdog()
    {
        const auto elapsed = this->now() - last_cmd_time_;
        if (elapsed > rclcpp::Duration(CMD_VEL_TIMEOUT))
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
        if(new_cmd_received)
        {
            RCLCPP_INFO(this->get_logger(), "Setting Motor Speeds: Left = %f, Right = %f", cmd_left_speed, cmd_right_speed);
            // Set Motor Speeds
            left_velocity.WithVelocity(units::angular_velocity::turns_per_second_t{cmd_left_speed});
            right_velocity.WithVelocity(units::angular_velocity::turns_per_second_t{cmd_right_speed});
            leftMotor.SetControl(left_velocity);
            rightMotor.SetControl(right_velocity);
            new_cmd_received = false;
        }
#endif
    }

    void calculate_and_publish_Odometry()
    {
        // Calculate Odometry based on wheel speeds and publish
        // This function can be implemented to calculate the robot's position and orientation
        // based on the wheel encoder readings and publish it as a nav_msgs::msg::Odometry message.
        const auto now = this->now();
        const double left_motor_turns = leftMotor.GetPosition().GetValueAsDouble();
        const double right_motor_turns = rightMotor.GetPosition().GetValueAsDouble();

        const double delta_left_motor_turns = unwrap_delta_turns(left_motor_turns, prev_left_motor_pos);
        const double delta_right_motor_turns = unwrap_delta_turns(right_motor_turns, prev_right_motor_pos);
        prev_left_motor_pos = left_motor_turns;
        prev_right_motor_pos = right_motor_turns;
        left_motor_pos_unwrapped += delta_left_motor_turns;
        right_motor_pos_unwrapped += delta_right_motor_turns;

        const double left_wheel_moved = delta_left_motor_turns * 2.0 * M_PI * WHEEL_DIAMETER/2.0 * GEAR_RATIO;
        const double right_wheel_moved = delta_right_motor_turns * 2.0 * M_PI * WHEEL_DIAMETER/2.0 * GEAR_RATIO;
        double linear_dist = (left_wheel_moved + right_wheel_moved) / 2.0;
        double angular_dist = (right_wheel_moved - left_wheel_moved) / WHEEL_DISTANCE;
        robot_pose.x += linear_dist * cos(robot_pose.theta + angular_dist / 2.0);
        robot_pose.y += linear_dist * sin(robot_pose.theta + angular_dist / 2.0);
        robot_pose.theta = wrap_angle_to_pi(robot_pose.theta + angular_dist);
        const double half_theta = robot_pose.theta / 2.0;
        const double left_wheel_speed_meas = leftMotor.GetVelocity().GetValueAsDouble() * 2.0 * M_PI * GEAR_RATIO;
        const double right_wheel_speed_meas = rightMotor.GetVelocity().GetValueAsDouble() * 2.0 * M_PI * GEAR_RATIO;
 
        // Publish Odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";
        odom_msg.pose.pose.position.x = robot_pose.x;
        odom_msg.pose.pose.position.y = robot_pose.y;
        odom_msg.pose.pose.orientation.z = std::sin(half_theta);
        odom_msg.pose.pose.orientation.w = std::cos(half_theta);
        odom_msg.twist.twist.linear.x = static_cast<float>((left_wheel_speed_meas + right_wheel_speed_meas) * WHEEL_DIAMETER / 4.0);
        odom_msg.twist.twist.angular.z = static_cast<float>((right_wheel_speed_meas - left_wheel_speed_meas) * WHEEL_DIAMETER / (2.0 * WHEEL_DISTANCE));

        // Diagonal covariance (off-diagonals remain 0, the message default).
        odom_msg.pose.covariance[0] = POSE_COV_XY;               // x
        odom_msg.pose.covariance[7] = POSE_COV_XY;               // y
        odom_msg.pose.covariance[14] = POSE_COV_Z_ROLL_PITCH;    // z
        odom_msg.pose.covariance[21] = POSE_COV_Z_ROLL_PITCH;    // roll
        odom_msg.pose.covariance[28] = POSE_COV_Z_ROLL_PITCH;    // pitch
        odom_msg.pose.covariance[35] = POSE_COV_YAW;             // yaw
        odom_msg.twist.covariance[0] = TWIST_COV_VX;                        // vx
        odom_msg.twist.covariance[7] = TWIST_COV_VY_VZ_ROLLRATE_PITCHRATE;  // vy
        odom_msg.twist.covariance[14] = TWIST_COV_VY_VZ_ROLLRATE_PITCHRATE; // vz
        odom_msg.twist.covariance[21] = TWIST_COV_VY_VZ_ROLLRATE_PITCHRATE; // roll rate
        odom_msg.twist.covariance[28] = TWIST_COV_VY_VZ_ROLLRATE_PITCHRATE; // pitch rate
        odom_msg.twist.covariance[35] = TWIST_COV_YAW_RATE;                 // yaw rate

        motor_odom_publisher_->publish(odom_msg);

        // Publish Motor Speeds and Joint States
        geometry_msgs::msg::Twist msg;
        msg.linear.x = static_cast<float>(leftMotor.GetVelocity().GetValueAsDouble());
        msg.linear.y = static_cast<float>(rightMotor.GetVelocity().GetValueAsDouble());
        motor_speed_publisher_->publish(msg);

        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = now;
        joint_msg.name = {"drivewhl_l_joint", "drivewhl_r_joint"};
        joint_msg.position = {
            left_motor_pos_unwrapped * 2.0 * M_PI * GEAR_RATIO,
            right_motor_pos_unwrapped * 2.0 * M_PI * GEAR_RATIO
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
