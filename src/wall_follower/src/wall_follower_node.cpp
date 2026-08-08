#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class WallFollower : public rclcpp::Node
{
public:
  WallFollower()
  : Node("wall_follower")
  {
    pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "/lidar_points");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    side_half_angle_deg_ = this->declare_parameter<double>("side_half_angle_deg", 25.0);
    min_valid_range_m_ = this->declare_parameter<double>("min_valid_range_m", 0.05);
    max_valid_range_m_ = this->declare_parameter<double>("max_valid_range_m", 30.0);
    linear_velocity_mps_ = this->declare_parameter<double>("linear_velocity_mps", 0.2);
    kp_ = this->declare_parameter<double>("kp", 1.2);
    ki_ = this->declare_parameter<double>("ki", 0.0);
    kd_ = this->declare_parameter<double>("kd", 0.08);
    max_angular_velocity_rps_ = this->declare_parameter<double>("max_angular_velocity_rps", 1.2);
    integral_limit_ = this->declare_parameter<double>("integral_limit", 2.0);
    z_min_ = this->declare_parameter<double>("z_min", -0.05);
    z_max_ = this->declare_parameter<double>("z_max", 0.2);
    filtered_pointcloud_topic_ = this->declare_parameter<std::string>(
      "filtered_pointcloud_topic", "/wall_follower/filtered_points");
    front_half_angle_deg_ = this->declare_parameter<double>("front_half_angle_deg", 20.0);
    controller_type_ = this->declare_parameter<std::string>("controller_type", "mpc");
    mpc_horizon_steps_ = this->declare_parameter<int>("mpc_horizon_steps", 15);
    mpc_dt_ = this->declare_parameter<double>("mpc_dt", 0.15);
    mpc_omega_candidates_ = this->declare_parameter<int>("mpc_omega_candidates", 21);
    mpc_weight_lateral_ = this->declare_parameter<double>("mpc_weight_lateral", 4.0);
    mpc_weight_heading_ = this->declare_parameter<double>("mpc_weight_heading", 1.0);
    mpc_weight_control_ = this->declare_parameter<double>("mpc_weight_control", 0.05);
    mpc_weight_control_rate_ = this->declare_parameter<double>("mpc_weight_control_rate", 0.1);
    mpc_weight_collision_ = this->declare_parameter<double>("mpc_weight_collision", 50.0);
    mpc_safety_margin_m_ = this->declare_parameter<double>("mpc_safety_margin_m", 0.3);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    filtered_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_pointcloud_topic_, rclcpp::QoS(10));

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&WallFollower::pointcloudCallback, this, std::placeholders::_1));

    wall_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() { this->timerCallback(); });

    prev_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Listening on %s and publishing PID cmd_vel to %s",
      pointcloud_topic_.c_str(),
      cmd_vel_topic_.c_str());
  }

  rclcpp::TimerBase::SharedPtr wall_timer_;

private:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kHalfPi = 1.57079632679489661923;

  static double deg2rad(const double deg)
  {
    return deg * kPi / 180.0;
  }

  static bool inAngleWindow(const double angle, const double center, const double half_width)
  {
    double diff = std::atan2(std::sin(angle - center), std::cos(angle - center));
    return std::fabs(diff) <= half_width;
  }
  
  void timerCallback()
  {
    // This callback is intentionally left empty. It serves as a placeholder to keep the node alive.

  }
  geometry_msgs::msg::Twist computePidCmdVel(const float left_distance, const float right_distance)
  {
    geometry_msgs::msg::Twist cmd;

    const bool left_valid = std::isfinite(left_distance);
    const bool right_valid = std::isfinite(right_distance);
    if (!(left_valid && right_valid)) {
      integral_error_ = 0.0;
      prev_error_ = 0.0;
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Missing side wall points (left_valid=%d, right_valid=%d); stopping robot",
        left_valid,
        right_valid);
      return cmd;
    }

    const rclcpp::Time now = this->now();
    double dt = (now - prev_time_).seconds();
    if (dt <= 1e-3) {
      dt = 1e-3;
    }
    prev_time_ = now;

    // Positive error means robot is closer to right wall and should turn left.
    const double error = static_cast<double>(left_distance - right_distance);
    integral_error_ += error * dt;
    integral_error_ = std::clamp(integral_error_, -integral_limit_, integral_limit_);
    const double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    double angular_z = kp_ * error + ki_ * integral_error_ + kd_ * derivative;
    angular_z = std::clamp(angular_z, -max_angular_velocity_rps_, max_angular_velocity_rps_);

    cmd.linear.x = linear_velocity_mps_;
    cmd.angular.z = angular_z;
    return cmd;
  }

  void computeSectorMins(
    const float x, const float y, float & left_min, float & right_min, float & front_min)
  {
    const float range = std::hypot(x, y);
    if (range < min_valid_range_m_ || range > max_valid_range_m_) {
      return;
    }

    const double angle = std::atan2(y, x);
    const double side_half_angle = deg2rad(side_half_angle_deg_);
    const double front_half_angle = deg2rad(front_half_angle_deg_);

    if (inAngleWindow(angle, kHalfPi, side_half_angle)) {
      left_min = std::min(left_min, range);
    }
    if (inAngleWindow(angle, -kHalfPi, side_half_angle)) {
      right_min = std::min(right_min, range);
    }
    if (inAngleWindow(angle, 0.0, front_half_angle)) {
      front_min = std::min(front_min, range);
    }
  }

  // Receding-horizon controller: shoots candidate constant angular velocities through a
  // cross-track/heading error model derived from the filtered cloud's left/right/front
  // distances, and applies the first control of the lowest-cost candidate.
  geometry_msgs::msg::Twist computeMpcCmdVel(
    const float left_distance, const float right_distance, const float front_distance)
  {
    geometry_msgs::msg::Twist cmd;

    const bool left_valid = std::isfinite(left_distance);
    const bool right_valid = std::isfinite(right_distance);
    if (!(left_valid && right_valid)) {
      prev_mpc_omega_ = 0.0;
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Missing side wall points (left_valid=%d, right_valid=%d); stopping robot",
        left_valid,
        right_valid);
      return cmd;
    }

    const double e0 = static_cast<double>(left_distance - right_distance);
    const double v = linear_velocity_mps_;
    const double dt = mpc_dt_;
    const int horizon = std::max(1, mpc_horizon_steps_);
    const int n_candidates = std::max(3, mpc_omega_candidates_);

    double best_omega = 0.0;
    double best_cost = std::numeric_limits<double>::infinity();

    for (int i = 0; i < n_candidates; ++i) {
      const double t = (2.0 * i) / (n_candidates - 1) - 1.0;
      const double omega = t * max_angular_velocity_rps_;

      double e = e0;
      double theta = 0.0;
      double x = 0.0;
      double cost = 0.0;

      for (int k = 0; k < horizon; ++k) {
        theta += omega * dt;
        e -= v * std::sin(theta) * dt;
        x += v * std::cos(theta) * dt;

        cost += mpc_weight_lateral_ * e * e + mpc_weight_heading_ * theta * theta;

        if (std::isfinite(front_distance)) {
          const double margin = (front_distance - mpc_safety_margin_m_) - x;
          if (margin < 0.0) {
            cost += mpc_weight_collision_ * margin * margin;
          }
        }
      }

      cost += mpc_weight_control_ * omega * omega;
      cost += mpc_weight_control_rate_ * (omega - prev_mpc_omega_) * (omega - prev_mpc_omega_);

      if (cost < best_cost) {
        best_cost = cost;
        best_omega = omega;
      }
    }

    prev_mpc_omega_ = best_omega;

    cmd.linear.x = linear_velocity_mps_;
    cmd.angular.z = std::clamp(best_omega, -max_angular_velocity_rps_, max_angular_velocity_rps_);
    return cmd;
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    float left_min = std::numeric_limits<float>::infinity();
    float right_min = std::numeric_limits<float>::infinity();
    float front_min = std::numeric_limits<float>::infinity();

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> iter_ring(*msg, "ring");


    // Buffer of points that survive ground-plane (z_min_) and max-height (z_max_) filtering.
    std::vector<std::array<float, 5>> filtered_points;
    filtered_points.reserve(msg->width * msg->height);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_ring) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      const float intensity = *iter_intensity;
      const float ring = *iter_ring;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < z_min_ || z > z_max_) {
        continue;
      }

      filtered_points.push_back({x, y, z, intensity, ring});

      computeSectorMins(x, y, left_min, right_min, front_min);
    }

    publishFilteredPointcloud(msg, filtered_points);

    geometry_msgs::msg::Twist cmd = (controller_type_ == "pid")
      ? computePidCmdVel(left_min, right_min)
      : computeMpcCmdVel(left_min, right_min, front_min);

    cmd_vel_pub_->publish(cmd);

    const bool left_valid = std::isfinite(left_min);
    const bool right_valid = std::isfinite(right_min);
    RCLCPP_INFO(
      this->get_logger(),
      "Side distances [left, right]=[%.3f, %.3f] -> cmd_vel [vx=%.3f, wz=%.3f]",
      left_valid ? left_min : -1.0F,
      right_valid ? right_min : -1.0F,
      cmd.linear.x,
      cmd.angular.z);
  }

  void publishFilteredPointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
    const std::vector<std::array<float, 5>> & points)
  {
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header = msg->header;
    cloud->height = 1;
    cloud->is_bigendian = false;
    cloud->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.setPointCloud2Fields(5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(*cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(*cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> out_ring(*cloud, "ring");
    for (const auto & point : points) {
      *out_x = point[0];
      *out_y = point[1];
      *out_z = point[2];
      *out_intensity = point[3];
      *out_ring = point[4];
      ++out_x;
      ++out_y;
      ++out_z;
      ++out_intensity;
      ++out_ring;
    }

    filtered_pointcloud_ = downsampling(cloud);
    filtered_pointcloud_pub_->publish(*filtered_pointcloud_);
  }

  sensor_msgs::msg::PointCloud2::SharedPtr downsampling(const sensor_msgs::msg::PointCloud2::SharedPtr & input_msg) {
      // 1. Convert ROS message to PCL point cloud format
      pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
      pcl_conversions::toPCL(*input_msg, *cloud_blob);
  
      // 2. Initialize VoxelGrid filter
      pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud(cloud_blob);
      
      // Set voxel leaf size (x, y, z in meters)
      sor.setLeafSize(0.05f, 0.05f, 0.05f); 
      sor.filter(*cloud_filtered_blob);
  
      // 3. Convert back to ROS message
      sensor_msgs::msg::PointCloud2 output_msg;
      pcl_conversions::moveFromPCL(*cloud_filtered_blob, output_msg);
  
      // 4. Publish downsampled cloud
      return std::make_shared<sensor_msgs::msg::PointCloud2>(output_msg);
  }

  std::string pointcloud_topic_;
  std::string cmd_vel_topic_;
  std::string filtered_pointcloud_topic_;
  std::string controller_type_;
  double side_half_angle_deg_;
  double front_half_angle_deg_;
  double min_valid_range_m_;
  double max_valid_range_m_;
  double z_min_; // Minimum z would be below the robot base, so we can filter out ground points.
  double z_max_; // Maximum z would be 0.2 m above the robot base, so we can filter out ceiling points.
  double linear_velocity_mps_;
  double kp_;
  double ki_;
  double kd_;
  double max_angular_velocity_rps_;
  double integral_limit_;
  double integral_error_ {0.0};
  double prev_error_ {0.0};
  rclcpp::Time prev_time_;
  sensor_msgs::msg::PointCloud2::SharedPtr filtered_pointcloud_;

  int mpc_horizon_steps_;
  int mpc_omega_candidates_;
  double mpc_dt_;
  double mpc_weight_lateral_;
  double mpc_weight_heading_;
  double mpc_weight_control_;
  double mpc_weight_control_rate_;
  double mpc_weight_collision_;
  double mpc_safety_margin_m_;
  double prev_mpc_omega_ {0.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollower>());
  rclcpp::shutdown();
  return 0;
}
