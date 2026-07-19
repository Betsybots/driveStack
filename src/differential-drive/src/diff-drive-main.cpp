#include <cstdio>
#include <chrono> // For using 10ms
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp" //For Imu sensor message
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3.hpp" // For Imu angular Velocity Storage
#include <deque> // For deque
#include <cmath> // For abs() with floating-point numbers
#include <cstdlib> // // For abs() with integer numbers
#include <rclcpp/qos.hpp>

#define ENABLE_MOTORS 1
#if ENABLE_MOTORS
#include "ctre/phoenix6/TalonFX.hpp"
#endif
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp" // for FeedEnable
#include "differential-drive/pid.hpp"
#include "geometry_msgs/msg/twist.hpp" // For Imu angular Velocity Storage
#include "sensor_msgs/msg/joint_state.hpp" // For Imu angular Velocity Storage
#include "nav_msgs/msg/odometry.hpp" // For Imu angular Velocity Storage

using namespace std::chrono_literals;
#if ENABLE_MOTORS
using namespace ctre::phoenix6;
#endif

#define WHEEL_DISTANCE (double)0.42783125
#define WHEEL_DIAMETER (double)0.1524 //0.1905
#define GEAR_RATIO (double)0.2 //0.3
#define M_PI (double)3.141592

float lin_vel_kp = 1;
float lin_vel_ki = 0;
float lin_vel_kd = 0;
float ang_vel_kp = 1;
float ang_vel_ki = 0;
float ang_vel_kd = 0;
float yaw_angle_kp = 1;
float yaw_angle_ki = 0;
float yaw_angle_kd = 0;

constexpr char const *CANBUS_NAME = "can1";
constexpr double ENCODER_MIN_TURNS = -16384.0;
constexpr double ENCODER_MAX_TURNS = 16383.999755859375;
constexpr double ENCODER_WRAP_RANGE_TURNS = ENCODER_MAX_TURNS - ENCODER_MIN_TURNS;
 
struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

double convert_radians_to_180_degrees(double rad) {
    double deg = rad * (180.0 / M_PI);
    // Normalize to the -180 to 180 range
    deg = fmod(deg, 360.0);
    if (deg > 180.0) {
        deg -= 360.0;
    } else if (deg <= -180.0) {
        deg += 360.0;
    }
    return deg;
}


class DifferentialDrive : public rclcpp::Node
{
public:

    DifferentialDrive() : Node("differential_drive")
    {
        // PID settings
        
        // Yaw angle PID setting     
        //yaw_pid = new PID(0.15, 0.01, 0.0);
        //this->declare_parameter("kp", 0.15);
        //this->declare_parameter("ki", 0.03);
        //this->declare_parameter("kd", 0.0);
	    //this->declare_parameter("vel", 0.2);
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
    prev_left_motor_pos = leftMotor.GetPosition().GetValueAsDouble();
    prev_right_motor_pos = rightMotor.GetPosition().GetValueAsDouble();
    left_motor_pos_unwrapped = prev_left_motor_pos;
    right_motor_pos_unwrapped = prev_right_motor_pos;
#endif
        // Motor speed limits
        //max_motor_limit = 0.3;
        //min_motor_limit = -0.3;
        
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 5, std::bind(&DifferentialDrive::cmd_vel_callback, this, std::placeholders::_1));

        motor_speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive/motor_speeds", 10);
        motor_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_cmd_vel", 20);
        motor_pos_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        
        // Periodic Function to Drive
        timer_ = this->create_wall_timer(10ms, std::bind(&DifferentialDrive::run_periodic, this));
    }
    
    void setMaxMotorLimit(double limit) { max_motor_limit = limit; }
    void setMinMotorLimit(double limit) { min_motor_limit = limit; }
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
        cmd_left_speed = (2.0 * cmd->linear.x - WHEEL_DISTANCE * cmd->angular.z)/(double)(2.0 * M_PI * WHEEL_DIAMETER * GEAR_RATIO);
        cmd_right_speed = (2.0 * cmd->linear.x + WHEEL_DISTANCE * cmd->angular.z)/(double)(2.0 * M_PI * WHEEL_DIAMETER * GEAR_RATIO);
        new_cmd_received = true;
        RCLCPP_INFO(this->get_logger(), "Left_speed = %f right_speed = %f", cmd_left_speed, cmd_right_speed);
    }
    
    void setMotorSpeeds()
    {
        //RCLCPP_INFO(this->get_logger(), "Left_speed = %f right_speed = %f", left_speed, right_speed);
        // Left Motor speed limiting 
        //if (left_speed > max_motor_limit)
        //    left_speed = max_motor_limit;
        //else if (left_speed < min_motor_limit)
        //    left_speed = min_motor_limit;
         
        // Right Motor speed limiting
        //if (right_speed > max_motor_limit)
        //    right_speed = max_motor_limit;
        //else if (right_speed < min_motor_limit)
        //    right_speed = min_motor_limit; 
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
    
    void PublishMessages()
    {
        
        nav_msgs::msg::Odometry nav_msg;
        nav_msg.header.stamp = this->now();
        nav_msg.header.frame_id = "odom";
        nav_msg.child_frame_id = "base_footprint";
        double left_wheel_speed_meas = leftMotor.GetVelocity().GetValueAsDouble() * 2.0 * M_PI * GEAR_RATIO;
        double right_wheel_speed_meas = rightMotor.GetVelocity().GetValueAsDouble() * 2.0 * M_PI * GEAR_RATIO;
        //RCLCPP_INFO(this->get_logger(), "Left_speed_meas = %f right_speed_meas = %f", left_speed_meas, right_speed_meas);
        nav_msg.twist.twist.linear.x = (float)(left_wheel_speed_meas + right_wheel_speed_meas) * (float)WHEEL_DIAMETER/4.0;
        nav_msg.twist.twist.angular.z = (float)(right_wheel_speed_meas - left_wheel_speed_meas) * WHEEL_DIAMETER/(float)(2.0 * WHEEL_DISTANCE);
        motor_odom_publisher_->publish(nav_msg);
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
        robot_pose.theta += angular_dist;
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
        setMotorSpeeds();
        calculate_and_publish_Odometry();  
    }

    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_filtered_publisher_;
    //rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr yaw_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_speed_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr motor_odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_pos_publisher_;
    
    double max_motor_limit;
    double min_motor_limit;
    
    double cmd_left_speed{0.0};
    double cmd_right_speed{0.0};

    double prev_left_motor_pos{0.0};
    double prev_right_motor_pos{0.0};
    double left_motor_pos_unwrapped{0.0};
    double right_motor_pos_unwrapped{0.0};

    double linear_distance;
    double angular_distance;
    Pose2D robot_pose;
    bool new_cmd_received = false;
    //int count;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DifferentialDrive>());
  rclcpp::shutdown();
  return 0;
}



/*

    float cmd_lin_spd;
    float cmd_ang_spd;
    float yaw_angle;
    PID* yaw_pid;

    float desired_yaw;
    bool drive_straight;
    bool speed_reduced;
    int straight_count_;
    int turn_count_;

    
    float reduceSpeedIfYawErrorIsMore(double yaw_error, float cmd_vel)
    {
        float reduced_spd = cmd_vel;
        //if (yaw_error < 0.174533 ) cmd_speed = cmd_vel;
        if (!speed_reduced && yaw_error > 0.174533) // 10 degrees
        {
            if (cmd_vel > 0.1)
            {
                reduced_spd = 0.1;
                speed_reduced = true;
            }
        }
        if (speed_reduced)
        {
            reduced_spd += (cmd_vel - 0.1)/500;
        }
        return reduced_spd;
    }
    
    void cmd_spd_regulator(float& cmd_speed, float cmd_vel, std::string tag)
    {
        float inc = (cmd_vel - cmd_speed)/50;
        cmd_speed += inc;
        //RCLCPP_INFO(this->get_logger(), "%s: Command Speed = %f, command Velocity = %f", tag.c_str(), cmd_speed, cmd_vel);     
    }
    
    void drive()
    {
        ctre::phoenix::unmanaged::FeedEnable(100);
        
        yaw_pid->setKP(this->get_parameter("kp").as_double());
        yaw_pid->setKI(this->get_parameter("ki").as_double());
        yaw_pid->setKD(this->get_parameter("kd").as_double());
        cmd_spd_regulator(cmd_lin_spd, cmd_lin_vel, "Linear");
        cmd_spd_regulator(cmd_ang_spd, cmd_ang_vel, "Angular");
        if (abs(cmd_ang_spd) < 0.01)
        {
            // Wait for 25 counts or 250ms to get the stable yaw angle 
            // Reason for 250ms wait is the window size used for averaging in 
            // Sensor Fusion Module
            if (!drive_straight)
            {
                if (straight_count_ < 25)
                    straight_count_++;
                else
                {
                   drive_straight = true;
                   straight_count_ = 0;
                   desired_yaw = yaw_angle;
                }
            }
            else // Once drive_straight is true, you have the stable yaw angle
            {
                double yaw_error = (double)(desired_yaw - yaw_angle);
                double pid_motor_correction = yaw_pid->getErrorOutput(yaw_error);
                cmd_lin_spd = reduceSpeedIfYawErrorIsMore(yaw_error, cmd_lin_spd);
                //RCLCPP_INFO(this->get_logger(), "yaw_error = %f pid_motor_correction = %f cmd_lin_spd = %f", yaw_error, pid_motor_correction, cmd_lin_spd);
                double left_speed = cmd_lin_spd + pid_motor_correction;
                double right_speed = cmd_lin_spd - pid_motor_correction;
                setMotorSpeeds(left_speed, right_speed);
            }
        }
        else
        {
            if (drive_straight)
            {
               if (turn_count_ < 10)
                    turn_count_++;
                else
                {
                   drive_straight = false;
                   turn_count_ = 0;
                } 
            }
            else
            {
                double left_speed = cmd_lin_spd + (double)(cmd_ang_spd * WHEEL_DISTANCE)/4.0;
                double right_speed = cmd_lin_spd - (double)(cmd_ang_spd * WHEEL_DISTANCE)/4.0;
                setMotorSpeeds(left_speed, right_speed);
            }
        }        
    }
    
*/
