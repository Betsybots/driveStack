// ============================================================
//  ACEINNA MTLT335D — ROS 2 Humble CAN IMU Driver
//  Source: imu_node.cpp
// ============================================================

#include "imu_can_interface/imu_node.hpp"
#include <stdexcept>

using namespace std::chrono_literals;

namespace {

uint32_t unpackUnsignedLE(const uint8_t* data, int start_bit, int bit_count)
{
  uint32_t value = 0U;
  for (int i = 0; i < bit_count; ++i) {
    const int bit_index = start_bit + i;
    const int byte_index = bit_index / 8;
    const int bit_in_byte = bit_index % 8;
    const uint32_t bit = (static_cast<uint32_t>(data[byte_index]) >> bit_in_byte) & 0x1U;
    value |= (bit << i);
  }
  return value;
}

// J1939 bit numbering is 1-based, LSB-first: pair_index 0 = bits 1-2 (the two
// least-significant bits of the byte), 1 = bits 3-4, 2 = bits 5-6, 3 = bits 7-8.
uint8_t fomBits(uint8_t byte, int pair_index)
{
  return (byte >> (pair_index * 2)) & 0x03U;
}

// FOM values per manual (Table 71/76/81): 00=Fully Functional, 01=Degraded,
// 10=Error, 11=N/A. Momentary "Degraded" is explicitly OK to ignore per the
// manual; only Error/N/A are rejected here.
bool fomOk(uint8_t fom)
{
  return fom == 0b00U || fom == 0b01U;
}

}  // namespace

// ════════════════════════════════════════════════════════
//  CONSTRUCTOR
// ════════════════════════════════════════════════════════

Mtlt335dCanNode::Mtlt335dCanNode()
: Node("imu_node")
{
  // ── 1. load all YAML params ──
  declareParameters();
  loadParameters();

  // ── 2. publishers ──
  imu_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_raw_, queue_size_);

  imu_filtered_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_filtered_, queue_size_);

  //tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // ── 3. open CAN socket ──
  if (!openCanSocket()) 
  {
    RCLCPP_FATAL(this->get_logger(),
        "Failed to open CAN socket on %s", can_interface_.c_str());
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(),
      "MTLT335D started | %s | SA=0x%02X | rate=%.0f Hz",
      can_interface_.c_str(), source_address_, publish_rate_hz_);

  // ── Publish timer (decoupled from CAN rate) ──
  last_publish_time_ = this->now();
  double period_ms = 1000.0 / publish_rate_hz_;
  publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period_ms)),
      std::bind(&Mtlt335dCanNode::publishImu, this));

  RCLCPP_INFO(this->get_logger(), "CAN thread started | publishing at %.0f Hz",
      publish_rate_hz_);
}

void Mtlt335dCanNode::declareParameters()
{
  // ── CAN ──
  this->declare_parameter<std::string>("can.interface",      "can1");
  this->declare_parameter<int>        ("can.bitrate",        250000);
  this->declare_parameter<int>        ("can.source_address", 0x82);
  this->declare_parameter<int>        ("can.can_id.accs",             0x08F02D82);
  this->declare_parameter<int>        ("can.can_id.hr_angular_rate",  0x0CF02A82);
  this->declare_parameter<int>        ("can.can_id.slope_sensor",     0x0CF02982);

  // ── ROS ──
  this->declare_parameter<std::string>("ros.frame_id",            "imu_link");
  this->declare_parameter<std::string>("ros.topic_raw",           "/imu/data_raw");
  this->declare_parameter<std::string>("ros.topic_filtered",      "/imu/data");
  this->declare_parameter<double>     ("ros.publish_rate_hz",     100.0);
  // this->declare_parameter<double>     ("ros.publish_rate_hz",     200.0);
  this->declare_parameter<int>        ("ros.queue_size",          10);
  this->declare_parameter<double>     ("ros.timestamp_offset_us", 0.0);
  this->declare_parameter<double>     ("ros.max_data_age_ms",     200.0);

  // ── Parsing — ACCS ──
  this->declare_parameter<int>   ("parsing.accs.bits_per_axis", 16);
  this->declare_parameter<double>("parsing.accs.scale",         0.01);
  this->declare_parameter<double>("parsing.accs.offset",       -320.0);

  // ── Parsing — HR Angular Rate ──
  this->declare_parameter<int>   ("parsing.hr_angular_rate.bits_per_axis", 19);
  this->declare_parameter<double>("parsing.hr_angular_rate.scale",         0.000976563);
  this->declare_parameter<double>("parsing.hr_angular_rate.offset",       -250.0);

  // ── Parsing — Slope Sensor ──
  this->declare_parameter<double>("parsing.slope_sensor.pitch_scale",   0.00003051758);
  this->declare_parameter<double>("parsing.slope_sensor.pitch_offset", -200.0);
  this->declare_parameter<double>("parsing.slope_sensor.roll_scale",    0.00003051758);
  this->declare_parameter<double>("parsing.slope_sensor.roll_offset",  -200.0);

  // ── Noise ──
  this->declare_parameter<double>("noise.gyro_noise_density",     2.909e-05);
  this->declare_parameter<double>("noise.gyro_bias_instability",  6.302e-06);
  this->declare_parameter<double>("noise.gyro_random_walk",       0.1);
  this->declare_parameter<double>("noise.accel_noise_density",    3.333e-04);
  this->declare_parameter<double>("noise.accel_bias_instability", 1.962e-04);
  this->declare_parameter<double>("noise.accel_random_walk",      0.02);

  // ── Covariance ──
    this->declare_parameter<bool>               ("covariance.derive_from_noise", false);
    this->declare_parameter<double>             ("covariance.effective_bandwidth_hz", 50.0);
  this->declare_parameter<std::vector<double>>("covariance.orientation",
      {-1.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>("covariance.angular_velocity",
      {8.46e-08, 0.0, 0.0,  0.0, 8.46e-08, 0.0,  0.0, 0.0, 8.46e-08});
  this->declare_parameter<std::vector<double>>("covariance.linear_acceleration",
      {1.111e-05, 0.0, 0.0,  0.0, 1.111e-05, 0.0,  0.0, 0.0, 1.111e-05});
}

void Mtlt335dCanNode::loadParameters()
{
  // ── CAN ──
  can_interface_   = this->get_parameter("can.interface").as_string();
  can_bitrate_     = this->get_parameter("can.bitrate").as_int();
  source_address_  = static_cast<uint8_t>(this->get_parameter("can.source_address").as_int());
  accs_can_id_     = static_cast<uint32_t>(this->get_parameter("can.can_id.accs").as_int());
  hr_ari_can_id_   = static_cast<uint32_t>(this->get_parameter("can.can_id.hr_angular_rate").as_int());
  slope_can_id_    = static_cast<uint32_t>(this->get_parameter("can.can_id.slope_sensor").as_int());

  // ── ROS ──
  frame_id_            = this->get_parameter("ros.frame_id").as_string();
  topic_raw_           = this->get_parameter("ros.topic_raw").as_string();
  topic_filtered_      = this->get_parameter("ros.topic_filtered").as_string();
  publish_rate_hz_     = this->get_parameter("ros.publish_rate_hz").as_double();
  queue_size_          = this->get_parameter("ros.queue_size").as_int();
  timestamp_offset_us_ = this->get_parameter("ros.timestamp_offset_us").as_double();
  max_data_age_ms_     = this->get_parameter("ros.max_data_age_ms").as_double();

  // ── Parsing — ACCS ──
  accs_bits_per_axis_ = this->get_parameter("parsing.accs.bits_per_axis").as_int();
  accs_scale_         = this->get_parameter("parsing.accs.scale").as_double();
  accs_offset_        = this->get_parameter("parsing.accs.offset").as_double();

  // ── Parsing — HR Angular Rate ──
  ari_bits_per_axis_  = this->get_parameter("parsing.hr_angular_rate.bits_per_axis").as_int();
  ari_scale_          = this->get_parameter("parsing.hr_angular_rate.scale").as_double();
  ari_offset_         = this->get_parameter("parsing.hr_angular_rate.offset").as_double();

  // ── Parsing — Slope Sensor ──
  slope_pitch_scale_  = this->get_parameter("parsing.slope_sensor.pitch_scale").as_double();
  slope_pitch_offset_ = this->get_parameter("parsing.slope_sensor.pitch_offset").as_double();
  slope_roll_scale_   = this->get_parameter("parsing.slope_sensor.roll_scale").as_double();
  slope_roll_offset_  = this->get_parameter("parsing.slope_sensor.roll_offset").as_double();

  // ── Noise ──
  gyro_noise_density_     = this->get_parameter("noise.gyro_noise_density").as_double();
  gyro_bias_instability_  = this->get_parameter("noise.gyro_bias_instability").as_double();
  gyro_random_walk_       = this->get_parameter("noise.gyro_random_walk").as_double();
  accel_noise_density_    = this->get_parameter("noise.accel_noise_density").as_double();
  accel_bias_instability_ = this->get_parameter("noise.accel_bias_instability").as_double();
  accel_random_walk_      = this->get_parameter("noise.accel_random_walk").as_double();

  cov_derive_from_noise_       = this->get_parameter("covariance.derive_from_noise").as_bool();
  cov_effective_bandwidth_hz_  = this->get_parameter("covariance.effective_bandwidth_hz").as_double();

  // ── Covariance → vector to std::array<double,9> ──
  auto v_ori  = this->get_parameter("covariance.orientation").as_double_array();
  auto v_gyro = this->get_parameter("covariance.angular_velocity").as_double_array();
  auto v_acc  = this->get_parameter("covariance.linear_acceleration").as_double_array();

  auto require_size = [this](const std::vector<double>& v, size_t expected, const char* name) {
    if (v.size() != expected) {
      throw std::runtime_error(
          std::string("Invalid size for ") + name +
          ": expected " + std::to_string(expected) +
          ", got " + std::to_string(v.size()));
    }
  };

  if (publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("ros.publish_rate_hz must be > 0");
  }
  if (max_data_age_ms_ <= 0.0) {
    throw std::runtime_error("ros.max_data_age_ms must be > 0");
  }
  if (accs_bits_per_axis_ != 16) {
    throw std::runtime_error("parsing.accs.bits_per_axis must be 16 (only PGN 61485 is wired up)");
  }
  if (ari_bits_per_axis_ != 16 && ari_bits_per_axis_ != 19) {
    throw std::runtime_error("parsing.hr_angular_rate.bits_per_axis must be 16 or 19");
  }

  require_size(v_ori, 9, "covariance.orientation");
  require_size(v_gyro, 9, "covariance.angular_velocity");
  require_size(v_acc, 9, "covariance.linear_acceleration");

  for (size_t i = 0; i < 9; ++i) {
    cov_orientation_[i]          = v_ori[i];
    cov_angular_velocity_[i]     = v_gyro[i];
    cov_linear_acceleration_[i]  = v_acc[i];
  }

  if (cov_derive_from_noise_) {
    const double bw_hz = std::max(cov_effective_bandwidth_hz_, 0.0);
    const double gyro_var = std::pow(gyro_noise_density_ * std::sqrt(bw_hz), 2.0);
    const double accel_var = std::pow(accel_noise_density_ * std::sqrt(bw_hz), 2.0);

    cov_angular_velocity_ = {
      gyro_var, 0.0,      0.0,
      0.0,      gyro_var, 0.0,
      0.0,      0.0,      gyro_var
    };

    cov_linear_acceleration_ = {
      accel_var, 0.0,       0.0,
      0.0,      accel_var,  0.0,
      0.0,      0.0,       accel_var
    };

    RCLCPP_INFO(this->get_logger(),
        "Covariance derived from noise density with effective bandwidth %.3f Hz | gyro var %.3e | accel var %.3e",
        bw_hz, gyro_var, accel_var);
  }

  RCLCPP_INFO(this->get_logger(), "All parameters loaded from YAML");
}

bool Mtlt335dCanNode::openCanSocket()
{
  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "socket() failed: %s", strerror(errno));
    return false;
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(this->get_logger(), "ioctl() failed: %s — is %s up?",
        strerror(errno), can_interface_.c_str());
    close(sock_); sock_ = -1;
    return false;
  }

  struct sockaddr_can addr{};
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "bind() failed: %s", strerror(errno));
    close(sock_); sock_ = -1;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "CAN socket opened on %s", can_interface_.c_str());

  // Bound recvmsg() so canReadLoop() periodically re-checks running_ instead
  // of blocking forever on a silent bus — otherwise the destructor's
  // can_thread_.join() can hang indefinitely on shutdown.
  struct timeval rx_timeout{};
  rx_timeout.tv_sec  = 0;
  rx_timeout.tv_usec = 500000;  // 500 ms
  if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &rx_timeout, sizeof(rx_timeout)) < 0) {
    RCLCPP_WARN(this->get_logger(), "setsockopt(SO_RCVTIMEO) failed: %s", strerror(errno));
  }

  // Enable kernel RX timestamping (SO_TIMESTAMP) so each frame's arrival
  // time is captured by the kernel, not this thread's scheduling time —
  // wall-clock based, so it stays valid under PTP sync with other hosts.
  const int enable_ts = 1;
  if (setsockopt(sock_, SOL_SOCKET, SO_TIMESTAMP, &enable_ts, sizeof(enable_ts)) < 0) {
    RCLCPP_WARN(this->get_logger(),
        "setsockopt(SO_TIMESTAMP) failed: %s — falling back to software receive time",
        strerror(errno));
  }

  return true;
}

void Mtlt335dCanNode::canReadLoop()
{
  RCLCPP_INFO(this->get_logger(), "CAN read thread started");

  struct can_frame frame{};
  struct iovec iov{};
  iov.iov_base = &frame;
  iov.iov_len  = sizeof(frame);

  // ancillary buffer sized to hold one SO_TIMESTAMP (struct timeval) cmsg
  char control[CMSG_SPACE(sizeof(struct timeval))];

  struct msghdr msg{};
  msg.msg_iov    = &iov;
  msg.msg_iovlen = 1;

  while (running_) {
    msg.msg_control    = control;
    msg.msg_controllen = sizeof(control);  // kernel overwrites this each call

    // blocking read — waits for next CAN frame
    int nbytes = recvmsg(sock_, &msg, 0);

    if (nbytes != sizeof(frame)) {
      continue;
    }

    // default to software time if SO_TIMESTAMP wasn't available/enabled
    int64_t stamp_ns = this->now().nanoseconds();
    for (struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
      if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
        struct timeval tv;
        std::memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
        stamp_ns = static_cast<int64_t>(tv.tv_sec) * 1000000000LL +
                   static_cast<int64_t>(tv.tv_usec) * 1000LL;
        break;
      }
    }

    uint32_t id = frame.can_id & CAN_EFF_MASK;

    // lock → parse → auto-unlock
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (id == accs_can_id_) {
      parseAccs(frame.data, stamp_ns);
    }
    else if (id == hr_ari_can_id_) {
      parseAngularRate(frame.data, stamp_ns);
    }
    else if (id == slope_can_id_) {
      parseSlopeSensor(frame.data, stamp_ns);
    }
  }

  RCLCPP_INFO(this->get_logger(), "CAN read thread stopped");
}

void Mtlt335dCanNode::parseAccs(const uint8_t* data, int64_t stamp_ns)
{
  // 8 bytes, little-endian packed fields, configurable bit width.
  const int bits = accs_bits_per_axis_;
  const uint32_t raw_y = unpackUnsignedLE(data, 0, bits);
  const uint32_t raw_x = unpackUnsignedLE(data, bits, bits);
  const uint32_t raw_z = unpackUnsignedLE(data, bits * 2, bits);

  // byte 7 (data[6]): FOM per axis — reject Error/N/A rather than publish it.
  const uint8_t fom_y = fomBits(data[6], 0);  // Lateral
  const uint8_t fom_x = fomBits(data[6], 1);  // Longitudinal
  const uint8_t fom_z = fomBits(data[6], 2);  // Vertical
  if (!fomOk(fom_y) || !fomOk(fom_x) || !fomOk(fom_z)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "ACCS FOM degraded/error (Y=%u X=%u Z=%u) — dropping frame", fom_y, fom_x, fom_z);
    return;
  }

  // engineering: raw × scale + offset  (m/s²)
  // Physical mount: sensor's "vertical" (raw_z) channel reads ~0 g and its
  // "lateral" (raw_y) channel reads gravity → sensor is rotated +90° about X
  // relative to the vehicle frame. Correct with a proper (right-handed)
  // rotation so Z ends up "up": x'=x, y'=-z, z'=y.
  accel_x_ =   raw_x * accs_scale_ + accs_offset_;
  accel_y_ =   -(raw_z * accs_scale_ + accs_offset_);   // raw_y * accs_scale_ + accs_offset_; 
  accel_z_ =    raw_y * accs_scale_ + accs_offset_;     // (raw_z * accs_scale_ + accs_offset_);      
  accel_ok_ = true;
  last_accel_ns_ = stamp_ns;
}


void Mtlt335dCanNode::parseAngularRate(const uint8_t* data, int64_t stamp_ns)
{
  // PGN 61482 — Standard Angular Rate
  // 8 bytes, little-endian packed fields, configurable bit width.
  // scale/offset come from YAML parsing.hr_angular_rate.*
  const int bits = ari_bits_per_axis_;
  const uint32_t raw_y = unpackUnsignedLE(data, 0, bits);           // pitch
  const uint32_t raw_x = unpackUnsignedLE(data, bits, bits);         // roll
  const uint32_t raw_z = unpackUnsignedLE(data, bits * 2, bits);     // yaw

  // byte 7 (data[6]): FOM per axis — reject Error/N/A rather than publish it.
  const uint8_t fom_pitch = fomBits(data[6], 0);
  const uint8_t fom_roll  = fomBits(data[6], 1);
  const uint8_t fom_yaw   = fomBits(data[6], 2);
  if (!fomOk(fom_pitch) || !fomOk(fom_roll) || !fomOk(fom_yaw)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Angular Rate FOM degraded/error (P=%u R=%u Y=%u) — dropping frame",
        fom_pitch, fom_roll, fom_yaw);
    return;
  }

  constexpr double DEG2RAD = M_PI / 180.0;

  const double roll_rate  = (raw_x * ari_scale_ + ari_offset_) * DEG2RAD;  // Roll
  const double pitch_rate = (raw_y * ari_scale_ + ari_offset_) * DEG2RAD;  // Pitch
  const double yaw_rate   = (raw_z * ari_scale_ + ari_offset_) * DEG2RAD;  // Yaw

  // Same physical mount correction as parseAccs(): sensor is rotated +90°
  // about X relative to the vehicle frame, so apply x'=x, y'=-z, z'=y.
  gyro_x_ =  roll_rate;
  gyro_y_ = -yaw_rate;
  gyro_z_ =  pitch_rate;
  gyro_ok_ = true;
  last_gyro_ns_ = stamp_ns;
}


void Mtlt335dCanNode::parseSlopeSensor(const uint8_t* data, int64_t stamp_ns)
{
  // MISMATCH (kept for reference): read as two 32-bit LE fields. Per the
  // MTLT335D manual (SSI2, PGN 61481) each axis is only 24 bits — byte 7 is
  // FOM/compensation status and byte 8 is a rolling latency counter, not
  // part of the angle. Reading them as the MSB caused ~512°-multiple jumps
  // whenever those status/latency bytes changed ("dancing" TF).
  // uint32_t raw_pitch = data[0] | (data[1] << 8) |
  //                      (data[2] << 16) | (data[3] << 24);
  // uint32_t raw_roll  = data[4] | (data[5] << 8) |
  //                      (data[6] << 16) | (data[7] << 24);

  // 24-bit per axis, little-endian: [Pitch(0-2), Roll(3-5), FOM(6), Latency(7)]
  uint32_t raw_pitch = data[0] | (data[1] << 8) | (data[2] << 16);
  uint32_t raw_roll  = data[3] | (data[4] << 8) | (data[5] << 16);

  // byte 7 (data[6]): compensation (bits 1-2, 5-6) + FOM (bits 3-4, 7-8) —
  // reject Error/N/A rather than publish it.
  const uint8_t fom_pitch = fomBits(data[6], 1);
  const uint8_t fom_roll  = fomBits(data[6], 3);
  if (!fomOk(fom_pitch) || !fomOk(fom_roll)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Slope Sensor FOM degraded/error (Pitch=%u Roll=%u) — dropping frame",
        fom_pitch, fom_roll);
    return;
  }

  // engineering: raw × scale + offset  (degrees)
  pitch_deg_ = raw_pitch * slope_pitch_scale_ + slope_pitch_offset_;
  roll_deg_  = raw_roll  * slope_roll_scale_  + slope_roll_offset_;
  slope_ok_ = true;
  last_slope_ns_ = stamp_ns;
}

void Mtlt335dCanNode::publishImu()
{
  // grab data under lock
  double ax, ay, az;
  double gx, gy, gz;
  double pitch, roll;
  bool has_accel, has_gyro, has_slope;
  int64_t accel_stamp_ns, gyro_stamp_ns;
  const int64_t now_ns = this->now().nanoseconds();
  const int64_t max_age_ns = static_cast<int64_t>(max_data_age_ms_ * 1e6);

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ax = accel_x_;  
    ay = accel_y_;  
    az = accel_z_;
    gx = gyro_x_;   
    gy = gyro_y_;   
    gz = gyro_z_;
    pitch = pitch_deg_;  
    roll = roll_deg_;
    accel_stamp_ns = last_accel_ns_;
    gyro_stamp_ns  = last_gyro_ns_;
    auto is_fresh = [now_ns, max_age_ns](int64_t stamp_ns) {
      return stamp_ns > 0 && now_ns >= stamp_ns && (now_ns - stamp_ns) <= max_age_ns;
    };
    has_accel = accel_ok_ && is_fresh(last_accel_ns_);
    has_gyro  = gyro_ok_  && is_fresh(last_gyro_ns_);
    has_slope = slope_ok_ && is_fresh(last_slope_ns_);
  }
  // mutex released here — rest is lock-free

  if (!has_accel || !has_gyro) 
  {
    return;
  }

  // Use the actual CAN-frame arrival time (newer of the two fused
  // channels), not this publish tick — keeps header.stamp valid for
  // PTP-synced cross-host fusion (LiDAR/SLAM on the Jetson).
  rclcpp::Time stamp = getTimestamp(std::max(accel_stamp_ns, gyro_stamp_ns));
  

  // ── RAW message (/imu/data_raw) ──
  sensor_msgs::msg::Imu raw;
  raw.header.stamp    = stamp;
  raw.header.frame_id = frame_id_;

  // orientation unknown → cov[0] = -1 (REP-145)
  raw.orientation.w = 1.0;
  std::copy(cov_orientation_.begin(), cov_orientation_.end(),
            raw.orientation_covariance.begin());

  raw.angular_velocity.x = gx;
  raw.angular_velocity.y = gy;
  raw.angular_velocity.z = gz;
  std::copy(cov_angular_velocity_.begin(), cov_angular_velocity_.end(),
            raw.angular_velocity_covariance.begin());

  raw.linear_acceleration.x = ax;
  raw.linear_acceleration.y = ay;
  raw.linear_acceleration.z = az;
  std::copy(cov_linear_acceleration_.begin(), cov_linear_acceleration_.end(),
            raw.linear_acceleration_covariance.begin());

  imu_raw_pub_->publish(raw);

  // ── FILTERED message (/imu/data) — only when slope available ──
  sensor_msgs::msg::Imu filt;
  if (has_slope)
  {
    filt.header.stamp    = stamp;
    filt.header.frame_id = frame_id_;

    // Slope-sensor pitch/roll are reported in the sensor's native mounting
    // frame (same as the raw accel/gyro axes before correction). Rotate the
    // resulting quaternion into the vehicle frame with the same +90° about
    // X mount correction applied to accel/gyro, so imu_link's TF orientation
    // is consistent with "Z up" in world instead of the native tilt frame.
    tf2::Quaternion q_native;
    q_native.setRPY(roll * M_PI / 180.0, pitch * M_PI / 180.0, 0.0);

    static const tf2::Quaternion kMountCorrection(0.70710678118654752, 0.0, 0.0, 0.70710678118654752);  // +90° about X
    tf2::Quaternion q = kMountCorrection * q_native * kMountCorrection.inverse();
    q.normalize();

    filt.orientation.x = q.x();
    filt.orientation.y = q.y();
    filt.orientation.z = q.z();
    filt.orientation.w = q.w();

    // REP-145: element [0] is the only cell consumers check to decide if
    // orientation is valid at all — yaw is unmeasured (fixed 0), so the
    // whole quaternion must be marked unknown (matches raw/cov_orientation_),
    // not just yaw's own diagonal term.
    filt.orientation_covariance = {
        -1.0, 0.0, 0.0,
        0.0,  0.0, 0.0,
        0.0,  0.0, 0.0
    };

    filt.angular_velocity.x = gx;
    filt.angular_velocity.y = gy;
    filt.angular_velocity.z = gz;
    std::copy(cov_angular_velocity_.begin(), cov_angular_velocity_.end(),
              filt.angular_velocity_covariance.begin());

    filt.linear_acceleration.x = ax;
    filt.linear_acceleration.y = ay;
    filt.linear_acceleration.z = az;
    std::copy(cov_linear_acceleration_.begin(), cov_linear_acceleration_.end(),
              filt.linear_acceleration_covariance.begin());

    imu_filtered_pub_->publish(filt);

    // ── live TF (world -> frame_id_), only when real orientation is available ──
    // (skipped, rather than falling back to identity, to avoid snapping/flicker)
    broadcastTf(filt);
  }

}

void Mtlt335dCanNode::broadcastTf(const sensor_msgs::msg::Imu& msg)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp    = msg.header.stamp;
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id  = frame_id_;

  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation      = msg.orientation;

  //tf_broadcaster_->sendTransform(tf_msg);
}

rclcpp::Time Mtlt335dCanNode::getTimestamp(int64_t sample_stamp_ns)
{
  rclcpp::Time t(sample_stamp_ns, this->now().get_clock_type());

  if (timestamp_offset_us_ != 0.0) {
    int64_t offset_ns = static_cast<int64_t>(timestamp_offset_us_ * 1000.0);
    t = rclcpp::Time(t.nanoseconds() + offset_ns, t.get_clock_type());
  }

  // Guard against duplicate / non-monotonic header stamps. sample_stamp_ns
  // comes from the kernel CAN RX timestamp (or this->now() as a fallback),
  // either of which can in principle repeat or go backward (clock step via
  // PTP/NTP/chrony, or two channels fused to the same instant). Downstream
  // localization (EKF/UKF) treats a repeated or backward-going stamp as
  // dt <= 0 and will drop the update or corrupt its covariance — so we
  // clamp to strictly increasing time here, once per publish cycle.
  // Use 1 us (not 1 ns) because some downstream code paths convert ROS time
  // to double seconds and can quantize sub-microsecond deltas at epoch scale.
  if (t <= last_publish_time_) {
    t = last_publish_time_ + rclcpp::Duration::from_nanoseconds(1000);  // bump by 1 us
  }
  last_publish_time_ = t;

  return t;
}

void Mtlt335dCanNode::startCanThread()
{
  running_ = true;
  can_thread_ = std::thread(&Mtlt335dCanNode::canReadLoop, this);  // ← HERE
  RCLCPP_INFO(this->get_logger(), "CAN read thread started");
}

Mtlt335dCanNode::~Mtlt335dCanNode()
{
  running_ = false;
  if (can_thread_.joinable()) {
    can_thread_.join();     // ← HERE
  }
  if (sock_ >= 0) {
    close(sock_);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mtlt335dCanNode>();

  // start CAN read thread
  node->startCanThread();

  // ROS2 spin (handles publish_timer_)
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}