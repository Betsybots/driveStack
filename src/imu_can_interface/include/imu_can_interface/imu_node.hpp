#pragma once
// ============================================================
//  ACEINNA MTLT335D — ROS 2 Humble CAN IMU Driver
//  Header: imu_node.hpp
// ============================================================

// ── ROS 2 ──
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>


// ── SocketCAN / Linux ──
#include <sys/socket.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

// ── STL ──
#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>

// ============================================================
//  CLASS DEFINITION
// ============================================================

class Mtlt335dCanNode : public rclcpp::Node
{
public:
  Mtlt335dCanNode();
  ~Mtlt335dCanNode();
  void startCanThread();

private:

  // ============================================================
  //  METHODS
  // ============================================================

  // ── initialization ──
  void declareParameters();
  void loadParameters();

  // ── CAN handling (thread-ready) ──
  bool openCanSocket();
  void canReadLoop();

  // ── parsing ──
  // stamp_ns: kernel CAN RX timestamp (SO_TIMESTAMP), wall-clock/PTP-based,
  // captured at bus arrival — not when this thread happens to get scheduled.
  void parseAccs(const uint8_t* data, int64_t stamp_ns);
  void parseAngularRate(const uint8_t* data, int64_t stamp_ns);
  void parseSlopeSensor(const uint8_t* data, int64_t stamp_ns);

  // ── publishing ──
  void publishImu();
  void broadcastTf(const sensor_msgs::msg::Imu& msg);
  rclcpp::Time getTimestamp(int64_t sample_stamp_ns);

  // ============================================================
  //  YAML SECTION → MEMBER VARIABLES
  // ============================================================

  // ── 1. CAN config ──
  std::string can_interface_;
  int         can_bitrate_;
  uint8_t     source_address_;
  uint32_t    accs_can_id_;
  uint32_t    hr_ari_can_id_;
  uint32_t    slope_can_id_;
  int         sock_{-1};

  // ── 2. ROS config ──
  std::string frame_id_;
  std::string topic_raw_;
  std::string topic_filtered_;
  double      publish_rate_hz_;
  int         queue_size_;
  double      timestamp_offset_us_;
  double      max_data_age_ms_;

  // ── 3. Parsing — ACCS (PGN 61485) ──
  int         accs_bits_per_axis_;
  double      accs_scale_;
  double      accs_offset_;

  // ── 3. Parsing — HR Angular Rate (PGN 65387) ──
  int         ari_bits_per_axis_;
  double      ari_scale_;
  double      ari_offset_;

  // ── 3. Parsing — Slope Sensor (PGN 61481) ──
  double      slope_pitch_scale_;
  double      slope_pitch_offset_;
  double      slope_roll_scale_;
  double      slope_roll_offset_;

  // ── 4. Noise (for EKF / fusion tuning) ──
  double      gyro_noise_density_;
  double      gyro_bias_instability_;
  double      gyro_random_walk_;
  double      accel_noise_density_;
  double      accel_bias_instability_;
  double      accel_random_walk_;

  // ── 5. Covariance (row-major 3×3) ──
  bool        cov_derive_from_noise_;
  double      cov_effective_bandwidth_hz_;
  std::array<double, 9> cov_orientation_;
  std::array<double, 9> cov_angular_velocity_;
  std::array<double, 9> cov_linear_acceleration_;

  // ============================================================
  //  RUNTIME STATE (not from YAML)
  // ============================================================

  // ── threading ──
  std::thread can_thread_;
  std::mutex  data_mutex_;
  bool        running_{false};

  // ── parsed sensor data (protected by data_mutex_) ──
  double      accel_x_{0.0}, accel_y_{0.0}, accel_z_{0.0};
  double      gyro_x_{0.0},  gyro_y_{0.0},  gyro_z_{0.0};
  double      pitch_deg_{0.0}, roll_deg_{0.0};
  bool        accel_ok_{false};
  bool        gyro_ok_{false};
  bool        slope_ok_{false};
  int64_t     last_accel_ns_{0};
  int64_t     last_gyro_ns_{0};
  int64_t     last_slope_ns_{0};

  // ── timing ──
  rclcpp::Time last_publish_time_;

  // ── ROS publishers & timer ──
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_filtered_pub_;

  // ── live TF (world -> frame_id_) ──
  //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

}; // end class