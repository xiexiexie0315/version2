#include "nav_serial_gimbal.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <limits>
#include <shared_mutex>
#include <stdexcept>
#include <type_traits>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
constexpr uint8_t kSof = 0x5A;
constexpr uint8_t kEscape = 0x5B;
constexpr uint8_t kEscapedSof = 0x01;
constexpr uint8_t kEscapedEscape = 0x02;
constexpr uint8_t kImuId = 0x02;
constexpr uint8_t kJointStateId = 0x0C;
constexpr uint8_t kRobotCommandId = 0x01;
constexpr float kTwoPi = 6.28318530717958647692F;

struct __attribute__((packed)) HeaderFrame
{
  uint8_t sof;
  uint8_t len;
  uint8_t id;
  uint8_t crc;
};

struct __attribute__((packed)) ReceiveImuData
{
  HeaderFrame header;
  uint32_t timestamp;
  float yaw;
  float pitch;
  float roll;
  float yaw_velocity;
  float pitch_velocity;
  float roll_velocity;
  uint16_t crc;
};

struct __attribute__((packed)) ReceiveJointState
{
  HeaderFrame header;
  uint32_t timestamp;
  float pitch;
  float yaw;
  uint16_t crc;
};

struct __attribute__((packed)) SendRobotCommand
{
  HeaderFrame header;
  uint32_t timestamp;
  float vx;
  float vy;
  float wz;
  float chassis_roll;
  float chassis_pitch;
  float chassis_yaw;
  float leg_length;
  float gimbal_pitch;
  float gimbal_yaw;
  uint8_t fire;
  uint8_t fric_on;
  uint8_t tracking;
  uint16_t crc;
};

static_assert(std::is_standard_layout_v<HeaderFrame>);
static_assert(std::is_standard_layout_v<ReceiveImuData>);
static_assert(std::is_standard_layout_v<ReceiveJointState>);
static_assert(std::is_standard_layout_v<SendRobotCommand>);
static_assert(sizeof(HeaderFrame) == 4);
static_assert(sizeof(ReceiveImuData) == 34);
static_assert(sizeof(ReceiveJointState) == 18);
static_assert(sizeof(SendRobotCommand) == 49);
static_assert(offsetof(ReceiveImuData, timestamp) == 4);
static_assert(offsetof(ReceiveImuData, yaw) == 8);
static_assert(offsetof(ReceiveImuData, crc) == 32);
static_assert(offsetof(SendRobotCommand, timestamp) == 4);
static_assert(offsetof(SendRobotCommand, vx) == 8);
static_assert(offsetof(SendRobotCommand, gimbal_pitch) == 36);
static_assert(offsetof(SendRobotCommand, gimbal_yaw) == 40);
static_assert(offsetof(SendRobotCommand, fire) == 44);
static_assert(offsetof(SendRobotCommand, fric_on) == 45);
static_assert(offsetof(SendRobotCommand, tracking) == 46);
static_assert(offsetof(SendRobotCommand, crc) == 47);

template <typename T>
T decode(const std::vector<uint8_t> & bytes)
{
  T value{};
  if (bytes.size() != sizeof(T)) throw std::runtime_error("Unexpected serial frame length");
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}
}  // namespace

NavSerialGimbal::NavSerialGimbal(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  port_ = yaml["nav_serial_port"] ?
            yaml["nav_serial_port"].as<std::string>() : "/dev/ttyACM0";
  baud_ = yaml["nav_serial_baud"] ? yaml["nav_serial_baud"].as<uint32_t>() : 115200U;
  yaw_command_scale_ = yaml["nav_serial_yaw_scale"] ?
                         yaml["nav_serial_yaw_scale"].as<float>() : -57.2957795F;
  pitch_command_scale_ = yaml["nav_serial_pitch_scale"] ?
                           yaml["nav_serial_pitch_scale"].as<float>() : 57.2957795F;
  state_.bullet_speed = yaml["nav_serial_bullet_speed"] ?
                          yaml["nav_serial_bullet_speed"].as<float>() : 22.0F;
  if (yaml["nav_serial_imu_timeout_ms"]) {
    imu_timeout_ = std::chrono::milliseconds(yaml["nav_serial_imu_timeout_ms"].as<int>());
  }
  if (yaml["nav_serial_command_timeout_ms"]) {
    command_timeout_ =
      std::chrono::milliseconds(yaml["nav_serial_command_timeout_ms"].as<int>());
  }
  if (yaml["nav_serial_chassis_command_timeout_ms"]) {
    chassis_command_timeout_ =
      std::chrono::milliseconds(yaml["nav_serial_chassis_command_timeout_ms"].as<int>());
  }
  if (yaml["nav_serial_byte_stuffing"]) {
    byte_stuffing_ = yaml["nav_serial_byte_stuffing"].as<bool>();
  }
  if (yaml["nav_serial_yaw_relative_to_start"]) {
    yaw_relative_to_start_ = yaml["nav_serial_yaw_relative_to_start"].as<bool>();
  }
  if (yaml["nav_serial_manual_test"]) {
    manual_test_ = yaml["nav_serial_manual_test"].as<bool>();
  }
  if (yaml["nav_serial_manual_test_yaw"]) {
    manual_test_yaw_ = yaml["nav_serial_manual_test_yaw"].as<float>();
  }
  if (yaml["nav_serial_manual_test_pitch"]) {
    manual_test_pitch_ = yaml["nav_serial_manual_test_pitch"].as<float>();
  }
  if (yaml["nav_serial_reconnect_ms"]) {
    reconnect_interval_ =
      std::chrono::milliseconds(yaml["nav_serial_reconnect_ms"].as<int>());
  }

  if (
    port_.empty() || baud_ == 0 ||
    !std::isfinite(yaw_command_scale_) || !std::isfinite(pitch_command_scale_) ||
    std::abs(yaw_command_scale_) < 1e-6F || std::abs(pitch_command_scale_) < 1e-6F ||
    !std::isfinite(state_.bullet_speed) || state_.bullet_speed <= 0.0F ||
    !std::isfinite(manual_test_yaw_) || !std::isfinite(manual_test_pitch_) ||
    command_timeout_.count() <= 0 || chassis_command_timeout_.count() <= 0 ||
    imu_timeout_.count() <= 0 ||
    reconnect_interval_.count() <= 0) {
    throw std::runtime_error("Invalid nav_serial configuration");
  }

  serial_.setPort(port_);
  serial_.setBaudrate(baud_);
  auto serial_timeout = serial::Timeout::simpleTimeout(20);
  serial_.setTimeout(serial_timeout);
  openSerial();

  tools::logger()->info(
    "[NavSerialGimbal] port={} baud={} yaw/pitch scales=({:.7f}, {:.7f}), "
    "yaw_relative_to_start={}, bullet_speed={:.2f}, byte_stuffing={}, "
    "command_timeout={} ms, chassis_timeout={} ms, fire disabled",
    port_, baud_, yaw_command_scale_, pitch_command_scale_, yaw_relative_to_start_,
    state_.bullet_speed, byte_stuffing_, command_timeout_.count(),
    chassis_command_timeout_.count());
  if (!byte_stuffing_) {
    tools::logger()->warn(
      "[NavSerialGimbal] byte_stuffing=false does not match the checked newcommunication2.0 "
      "firmware");
  }
  if (manual_test_) {
    tools::logger()->warn(
      "[NavSerialGimbal] MANUAL TEST enabled: tracking=1, yaw={:.4f} rad -> {:.4f} wire, "
      "pitch={:.4f} rad -> {:.4f} wire. Keep the gimbal clear and disable this after the test",
      manual_test_yaw_, manual_test_yaw_ * yaw_command_scale_, manual_test_pitch_,
      manual_test_pitch_ * pitch_command_scale_);
  }

  receive_thread_ = std::thread(&NavSerialGimbal::receiveLoop, this);
  send_thread_ = std::thread(&NavSerialGimbal::sendLoop, this);

  std::unique_lock<std::mutex> lock(imu_mutex_);
  if (!imu_cv_.wait_for(lock, std::chrono::seconds(3), [this] {
        return quit_ || !imu_samples_.empty();
      })) {
    tools::logger()->warn(
      "[NavSerialGimbal] No valid IMU frame within 3 seconds. Camera preview will continue, "
      "and only tracking=0 safe commands will be transmitted");
  }
}

NavSerialGimbal::~NavSerialGimbal()
{
  try {
    if (!stopAndWait(std::chrono::milliseconds(250))) {
      tools::logger()->warn(
        "[NavSerialGimbal] Could not confirm a safe zero frame before shutdown");
    }
  } catch (const std::exception & e) {
    tools::logger()->warn(
      "[NavSerialGimbal] Safe shutdown request failed: {}", e.what());
  }
  quit_ = true;
  imu_cv_.notify_all();
  safe_command_cv_.notify_all();
  if (receive_thread_.joinable()) receive_thread_.join();
  if (send_thread_.joinable()) send_thread_.join();
  disconnectSerial("shutdown");
}

GimbalMode NavSerialGimbal::mode() const
{
  return GimbalMode::AUTO_AIM;
}

GimbalState NavSerialGimbal::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

bool NavSerialGimbal::imuFresh() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return connected_.load(std::memory_order_acquire) && have_imu_ &&
         std::chrono::steady_clock::now() - last_imu_time_ <= imu_timeout_;
}

std::string NavSerialGimbal::str(GimbalMode mode) const
{
  return mode == GimbalMode::AUTO_AIM ? "AUTO_AIM" : "IDLE";
}

Eigen::Quaterniond NavSerialGimbal::q(std::chrono::steady_clock::time_point timestamp)
{
  std::unique_lock<std::mutex> lock(imu_mutex_);
  imu_cv_.wait_for(lock, std::chrono::milliseconds(100), [this, timestamp] {
    return quit_ || (!imu_samples_.empty() && imu_samples_.back().timestamp >= timestamp);
  });

  if (imu_samples_.empty()) return Eigen::Quaterniond::Identity();
  if (timestamp <= imu_samples_.front().timestamp) return imu_samples_.front().q;
  if (timestamp >= imu_samples_.back().timestamp) return imu_samples_.back().q;

  for (size_t i = 1; i < imu_samples_.size(); ++i) {
    const auto & a = imu_samples_[i - 1];
    const auto & b = imu_samples_[i];
    if (timestamp > b.timestamp) continue;
    const auto interval = std::chrono::duration<double>(b.timestamp - a.timestamp).count();
    if (interval <= std::numeric_limits<double>::epsilon()) return b.q;
    const auto elapsed = std::chrono::duration<double>(timestamp - a.timestamp).count();
    return a.q.slerp(std::clamp(elapsed / interval, 0.0, 1.0), b.q).normalized();
  }
  return imu_samples_.back().q;
}

Eigen::Quaterniond NavSerialGimbal::latestQ() const
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (imu_samples_.empty()) return Eigen::Quaterniond::Identity();
  return imu_samples_.back().q;
}

void NavSerialGimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
  float pitch_vel, float pitch_acc)
{
  (void)fire;
  (void)yaw_vel;
  (void)yaw_acc;
  (void)pitch_vel;
  (void)pitch_acc;
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_tracking_ = control && std::isfinite(yaw) && std::isfinite(pitch);
  have_command_ = true;
  command_connection_generation_ =
    connection_generation_.load(std::memory_order_acquire);
  last_command_time_ = std::chrono::steady_clock::now();
  if (command_tracking_) {
    command_yaw_ = yaw;
    command_pitch_ = pitch;
  } else {
    command_yaw_ = 0.0F;
    command_pitch_ = 0.0F;
  }
}

void NavSerialGimbal::send(VisionToGimbal command)
{
  send(
    command.mode != 0, false, command.yaw, command.yaw_vel, command.yaw_acc,
    command.pitch, command.pitch_vel, command.pitch_acc);
}

void NavSerialGimbal::setChassisCommand(float vx, float vy, float wz)
{
  std::lock_guard<std::mutex> lock(chassis_command_mutex_);
  if (std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz)) {
    chassis_vx_ = vx;
    chassis_vy_ = vy;
    chassis_wz_ = wz;
    have_chassis_command_ = true;
  } else {
    chassis_vx_ = 0.0F;
    chassis_vy_ = 0.0F;
    chassis_wz_ = 0.0F;
    have_chassis_command_ = false;
    tools::logger()->error(
      "[SerialTX] rejected non-finite chassis command ({}, {}, {}); output forced to zero", vx,
      vy, wz);
  }
  chassis_command_connection_generation_ =
    connection_generation_.load(std::memory_order_acquire);
  last_chassis_command_time_ = std::chrono::steady_clock::now();
}

bool NavSerialGimbal::stopAndWait(std::chrono::milliseconds timeout)
{
  if (timeout.count() <= 0) return false;

  uint64_t request_generation = 0;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_tracking_ = false;
    command_yaw_ = 0.0F;
    command_pitch_ = 0.0F;
    manual_test_ = false;
    have_command_ = true;
    command_connection_generation_ =
      connection_generation_.load(std::memory_order_acquire);
    last_command_time_ = std::chrono::steady_clock::now();
    request_generation =
      safe_request_generation_.fetch_add(1, std::memory_order_acq_rel) + 1;
  }
  {
    std::lock_guard<std::mutex> lock(chassis_command_mutex_);
    chassis_vx_ = 0.0F;
    chassis_vy_ = 0.0F;
    chassis_wz_ = 0.0F;
    have_chassis_command_ = true;
    chassis_command_connection_generation_ =
      connection_generation_.load(std::memory_order_acquire);
    last_chassis_command_time_ = std::chrono::steady_clock::now();
  }

  std::unique_lock<std::mutex> lock(safe_command_mutex_);
  return safe_command_cv_.wait_for(lock, timeout, [this, request_generation] {
    return safe_sent_generation_.load(std::memory_order_acquire) >= request_generation;
  });
}

bool NavSerialGimbal::openSerial()
{
  if (connected_) return true;

  std::unique_lock<std::shared_mutex> lock(serial_lifecycle_mutex_);
  if (connected_) return true;
  try {
    if (serial_.isOpen()) serial_.close();
    serial_.open();
    serial_.flushInput();
    clearImuState();
    const auto generation =
      connection_generation_.fetch_add(1, std::memory_order_acq_rel) + 1;
    const auto count = reconnects_.fetch_add(1, std::memory_order_relaxed) + 1;
    connected_.store(true, std::memory_order_release);
    tools::logger()->info(
      "[NavSerialGimbal] Serial connected: {} at {} (open_count={}, generation={})", port_,
      baud_, count, generation);
    return true;
  } catch (const std::exception & e) {
    connected_ = false;
    tools::logger()->warn(
      "[NavSerialGimbal] Cannot open {} at {}: {}; retrying in {} ms", port_, baud_, e.what(),
      reconnect_interval_.count());
    return false;
  }
}

void NavSerialGimbal::disconnectSerial(
  const std::string & reason, uint64_t expected_generation)
{
  std::unique_lock<std::shared_mutex> lock(serial_lifecycle_mutex_);
  if (
    expected_generation != 0 &&
    connection_generation_.load(std::memory_order_acquire) != expected_generation) {
    return;
  }
  const bool was_connected = connected_.exchange(false);
  try {
    if (serial_.isOpen()) serial_.close();
  } catch (const std::exception & e) {
    if (!quit_) tools::logger()->warn("[NavSerialGimbal] Serial close failed: {}", e.what());
  }
  clearImuState();
  if (was_connected && !quit_) {
    tools::logger()->warn("[NavSerialGimbal] Serial disconnected: {}", reason);
  }
}

void NavSerialGimbal::clearImuState()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    have_imu_ = false;
    last_imu_time_ = {};
    state_.yaw = 0.0F;
    state_.yaw_vel = 0.0F;
    state_.pitch = 0.0F;
    state_.pitch_vel = 0.0F;
  }
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_samples_.clear();
  }
  imu_cv_.notify_all();
}

size_t NavSerialGimbal::readSerial(uint8_t * data, size_t size)
{
  std::shared_lock<std::shared_mutex> lock(serial_lifecycle_mutex_);
  if (!connected_ || !serial_.isOpen()) return 0;
  return serial_.read(data, size);
}

size_t NavSerialGimbal::writeSerial(
  const uint8_t * data, size_t size, uint64_t connection_generation)
{
  std::shared_lock<std::shared_mutex> lock(serial_lifecycle_mutex_);
  if (
    !connected_.load(std::memory_order_acquire) || !serial_.isOpen() ||
    connection_generation_.load(std::memory_order_acquire) != connection_generation) {
    return 0;
  }
  return serial_.write(data, size);
}

bool NavSerialGimbal::readExact(uint8_t * data, size_t size)
{
  size_t offset = 0;
  while (!quit_ && offset < size) {
    const auto received = readSerial(data + offset, size - offset);
    if (received == 0) {
      receive_timeouts_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    offset += received;
    received_bytes_.fetch_add(received, std::memory_order_relaxed);
  }
  return offset == size;
}

bool NavSerialGimbal::readDecodedExact(uint8_t * data, size_t size)
{
  if (!byte_stuffing_) return readExact(data, size);

  size_t offset = 0;
  while (!quit_ && offset < size) {
    uint8_t raw = 0;
    if (!readExact(&raw, 1)) return false;
    if (raw != kEscape) {
      data[offset++] = raw;
      continue;
    }

    uint8_t escaped = 0;
    if (!readExact(&escaped, 1)) return false;
    if (escaped == kEscapedSof)
      data[offset++] = kSof;
    else if (escaped == kEscapedEscape)
      data[offset++] = kEscape;
    else {
      invalid_escape_sequences_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
  }
  return offset == size;
}

size_t NavSerialGimbal::writeFrame(
  const uint8_t * data, size_t size, uint64_t connection_generation)
{
  std::vector<uint8_t> encoded;
  if (byte_stuffing_) {
    encoded.reserve(size * 2);
    encoded.push_back(data[0]);
    for (size_t i = 1; i < size; ++i) {
      if (data[i] == kSof) {
        encoded.push_back(kEscape);
        encoded.push_back(kEscapedSof);
      } else if (data[i] == kEscape) {
        encoded.push_back(kEscape);
        encoded.push_back(kEscapedEscape);
      } else {
        encoded.push_back(data[i]);
      }
    }
  } else {
    encoded.assign(data, data + size);
  }

  size_t offset = 0;
  while (!quit_ && offset < encoded.size()) {
    const auto written =
      writeSerial(encoded.data() + offset, encoded.size() - offset, connection_generation);
    if (written == 0) {
      if (
        !connected_.load(std::memory_order_acquire) ||
        connection_generation_.load(std::memory_order_acquire) != connection_generation) {
        return 0;
      }
      break;
    }
    offset += written;
  }
  if (offset != encoded.size()) {
    throw std::runtime_error("Incomplete serial frame write");
  }
  return offset;
}

void NavSerialGimbal::receiveLoop()
{
  while (!quit_) {
    if (!connected_) {
      if (!openSerial()) std::this_thread::sleep_for(reconnect_interval_);
      continue;
    }
    try {
      uint8_t sof = 0;
      if (!readExact(&sof, 1) || sof != kSof) continue;

      HeaderFrame header{};
      header.sof = sof;
      if (!readDecodedExact(reinterpret_cast<uint8_t *>(&header) + 1, sizeof(header) - 1)) continue;
      if (!tools::check_crc8(reinterpret_cast<uint8_t *>(&header), sizeof(header))) {
        ++crc8_errors_;
        continue;
      }
      ++valid_headers_;

      std::vector<uint8_t> frame(sizeof(header) + header.len + sizeof(uint16_t));
      std::memcpy(frame.data(), &header, sizeof(header));
      if (!readDecodedExact(frame.data() + sizeof(header), frame.size() - sizeof(header))) continue;
      if (!tools::check_crc16(frame.data(), frame.size())) {
        ++crc16_errors_;
        continue;
      }
      handleFrame(header.id, frame);
    } catch (const std::exception & e) {
      if (!quit_) disconnectSerial(e.what());
    }
  }
}

void NavSerialGimbal::handleFrame(uint8_t id, const std::vector<uint8_t> & frame)
{
  const auto now = std::chrono::steady_clock::now();
  if (id == kImuId && frame.size() != sizeof(ReceiveImuData)) {
    const auto count = unexpected_length_frames_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (count == 1 || count % 100 == 0) {
      tools::logger()->error(
        "[SerialRX] IMU length mismatch actual={} expected={} count={}", frame.size(),
        sizeof(ReceiveImuData), count);
    }
    return;
  }
  if (id == kJointStateId && frame.size() != sizeof(ReceiveJointState)) {
    const auto count = unexpected_length_frames_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (count == 1 || count % 100 == 0) {
      tools::logger()->error(
        "[SerialRX] joint length mismatch actual={} expected={} count={}", frame.size(),
        sizeof(ReceiveJointState), count);
    }
    return;
  }

  if (id == kImuId) {
    const auto packet = decode<ReceiveImuData>(frame);
    const float imu_yaw = packet.yaw;
    const float imu_pitch = packet.pitch;
    const float imu_roll = packet.roll;
    const float imu_yaw_velocity = packet.yaw_velocity;
    const float imu_pitch_velocity = packet.pitch_velocity;
    const float imu_roll_velocity = packet.roll_velocity;
    const uint32_t imu_timestamp = packet.timestamp;
    if (
      !std::isfinite(imu_yaw) || !std::isfinite(imu_pitch) ||
      !std::isfinite(imu_roll) || !std::isfinite(imu_yaw_velocity) ||
      !std::isfinite(imu_pitch_velocity) || !std::isfinite(imu_roll_velocity)) {
      ++invalid_imu_frames_;
      return;
    }
    const Eigen::Quaterniond quaternion =
      Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(imu_pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(imu_roll, Eigen::Vector3d::UnitX());
    if (
      !quaternion.coeffs().allFinite() ||
      quaternion.norm() <= std::numeric_limits<double>::epsilon()) {
      ++invalid_imu_frames_;
      return;
    }
    ++valid_imu_frames_;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      imu_samples_.push_back({quaternion.normalized(), now});
      while (imu_samples_.size() > 2000) imu_samples_.pop_front();
    }
    imu_cv_.notify_all();

    std::lock_guard<std::mutex> lock(state_mutex_);
    have_imu_ = true;
    last_imu_time_ = now;
    const bool controller_restarted =
      have_controller_timestamp_ && last_controller_timestamp_ < 0xF0000000U &&
      static_cast<uint64_t>(imu_timestamp) + 1000U < last_controller_timestamp_;
    if (controller_restarted) {
      yaw_reference_valid_ = false;
      tools::logger()->warn(
        "[NavSerialGimbal] Controller timestamp restarted ({} -> {}); resetting link state",
        last_controller_timestamp_, imu_timestamp);
    }
    have_controller_timestamp_ = true;
    last_controller_timestamp_ = imu_timestamp;
    if (yaw_relative_to_start_ && !yaw_reference_valid_) {
      yaw_reference_ = imu_yaw;
      yaw_reference_valid_ = true;
      tools::logger()->info(
        "[NavSerialGimbal] Captured yaw reference {:.6f} rad for relative command mode",
        yaw_reference_);
    }
    state_.yaw = imu_yaw;
    state_.pitch = imu_pitch;
    state_.yaw_vel = imu_yaw_velocity;
    state_.pitch_vel = imu_pitch_velocity;
  } else if (id == kJointStateId) {
    const auto packet = decode<ReceiveJointState>(frame);
    const float joint_pitch = packet.pitch;
    const float joint_yaw = packet.yaw;
    if (!std::isfinite(joint_pitch) || !std::isfinite(joint_yaw)) {
      ++invalid_joint_frames_;
      return;
    }
    ++valid_joint_frames_;
    // Navigation treats ID 0x0C as odometry-only. IMU remains the aiming state source.
  } else {
    const auto count = unknown_id_frames_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (count == 1 || count % 100 == 0) {
      tools::logger()->warn(
        "[SerialRX] valid frame ignored id=0x{:02X} bytes={} count={}", id, frame.size(),
        count);
    }
  }
}

void NavSerialGimbal::sendLoop()
{
  SendRobotCommand packet{};
  packet.header.sof = kSof;
  packet.header.id = kRobotCommandId;
  packet.header.len = sizeof(packet) - sizeof(HeaderFrame) - sizeof(packet.crc);
  packet.header.crc = tools::get_crc8(reinterpret_cast<uint8_t *>(&packet.header), 3);
  packet.fire = 0;
  packet.fric_on = 0;
  packet.tracking = 0;
  auto diagnostic_time = std::chrono::steady_clock::now();
  uint64_t previous_received_bytes = 0;
  uint64_t previous_valid_headers = 0;
  uint64_t previous_imu_frames = 0;
  uint64_t previous_transmitted_frames = 0;
  uint64_t previous_crc8_errors = 0;
  uint64_t previous_crc16_errors = 0;
  uint64_t previous_transmit_errors = 0;
  bool status_initialized = false;
  bool previous_fresh_imu = false;
  bool previous_fresh_command = false;
  bool previous_fresh_chassis_command = false;

  const uint8_t tx_header_sof = packet.header.sof;
  const uint8_t tx_header_len = packet.header.len;
  const uint8_t tx_header_id = packet.header.id;
  const uint8_t tx_header_crc = packet.header.crc;
  tools::logger()->info(
    "[SerialTX] raw_header={:02X} {:02X} {:02X} {:02X}, raw_frame_bytes={}, "
    "byte_stuffing={}",
    tx_header_sof, tx_header_len, tx_header_id, tx_header_crc, sizeof(packet),
    byte_stuffing_);

  while (!quit_) {
    const auto connection_generation =
      connection_generation_.load(std::memory_order_acquire);
    if (!connected_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    bool fresh_imu = false;
    bool yaw_reference_valid = false;
    float yaw_reference = 0.0F;
    float current_imu_yaw = 0.0F;
    const auto loop_time = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      fresh_imu = have_imu_ && loop_time - last_imu_time_ <= imu_timeout_;
      yaw_reference_valid = yaw_reference_valid_;
      yaw_reference = yaw_reference_;
      current_imu_yaw = state_.yaw;
    }
    bool fresh_command = false;
    bool fresh_chassis_command = false;
    uint64_t safe_request_generation = 0;
    if (connected_.load(std::memory_order_acquire)) {
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        fresh_command =
          have_command_ && loop_time - last_command_time_ <= command_timeout_ &&
          command_connection_generation_ == connection_generation;
        const bool active =
          fresh_imu && (manual_test_ || (command_tracking_ && fresh_command));
        float yaw_radians = 0.0F;
        float pitch_radians = 0.0F;
        if (active && manual_test_) {
          yaw_radians = manual_test_yaw_;
          pitch_radians = manual_test_pitch_;
        } else if (active) {
          yaw_radians = command_yaw_;
          if (yaw_relative_to_start_ && yaw_reference_valid) {
            yaw_radians = std::remainder(command_yaw_ - yaw_reference, kTwoPi);
          }
          pitch_radians = command_pitch_;
        }
        const float yaw_wire = yaw_radians * yaw_command_scale_;
        const float pitch_wire = pitch_radians * pitch_command_scale_;
        if (active && std::isfinite(yaw_wire) && std::isfinite(pitch_wire)) {
          packet.gimbal_yaw = yaw_wire;
          packet.gimbal_pitch = pitch_wire;
          packet.tracking = 1U;
        } else {
          packet.gimbal_yaw = 0.0F;
          packet.gimbal_pitch = 0.0F;
          packet.tracking = 0U;
        }
        safe_request_generation =
          safe_request_generation_.load(std::memory_order_acquire);
      }
      {
        std::lock_guard<std::mutex> lock(chassis_command_mutex_);
        const bool pending_safe_request =
          safe_sent_generation_.load(std::memory_order_acquire) < safe_request_generation;
        fresh_chassis_command =
          !pending_safe_request && have_chassis_command_ &&
          loop_time - last_chassis_command_time_ <= chassis_command_timeout_ &&
          chassis_command_connection_generation_ == connection_generation;
        if (fresh_chassis_command) {
          packet.vx = chassis_vx_;
          packet.vy = chassis_vy_;
          packet.wz = chassis_wz_;
        } else {
          packet.vx = 0.0F;
          packet.vy = 0.0F;
          packet.wz = 0.0F;
        }
      }
      packet.timestamp = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
      const auto crc = tools::get_crc16(
        reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - sizeof(packet.crc));
      packet.crc = crc;
      try {
        const auto wire_bytes = writeFrame(
          reinterpret_cast<uint8_t *>(&packet), sizeof(packet), connection_generation);
        if (wire_bytes == 0) continue;
        transmitted_raw_bytes_.fetch_add(sizeof(packet), std::memory_order_relaxed);
        transmitted_wire_bytes_.fetch_add(wire_bytes, std::memory_order_relaxed);
        transmitted_frames_.fetch_add(1, std::memory_order_relaxed);
        const bool sent_tracking = packet.tracking != 0U;
        const float sent_yaw = packet.gimbal_yaw;
        const float sent_pitch = packet.gimbal_pitch;
        if (
          !sent_tracking && sent_yaw == 0.0F && sent_pitch == 0.0F &&
          safe_sent_generation_.load(std::memory_order_acquire) < safe_request_generation) {
          safe_sent_generation_.store(safe_request_generation, std::memory_order_release);
          safe_command_cv_.notify_all();
        }
      } catch (const std::exception & e) {
        transmit_errors_.fetch_add(1, std::memory_order_relaxed);
        if (!quit_) disconnectSerial(e.what(), connection_generation);
      }
    }
    if (status_initialized) {
      if (fresh_imu != previous_fresh_imu) {
        if (fresh_imu)
          tools::logger()->info("[SerialRX] IMU stream recovered and is fresh");
        else
          tools::logger()->error(
            "[SerialRX] IMU stream became stale; self-aim tracking output is disabled");
      }
      if (fresh_command != previous_fresh_command) {
        if (fresh_command)
          tools::logger()->info("[SerialTX] self-aim planner command stream recovered");
        else
          tools::logger()->error(
            "[SerialTX] self-aim planner command became stale; tracking output is disabled");
      }
      if (fresh_chassis_command != previous_fresh_chassis_command) {
        if (fresh_chassis_command)
          tools::logger()->info("[SerialTX] navigation chassis command stream recovered");
        else
          tools::logger()->error(
            "[SerialTX] navigation chassis command became stale; vx/vy/wz forced to zero");
      }
    }
    status_initialized = true;
    previous_fresh_imu = fresh_imu;
    previous_fresh_command = fresh_command;
    previous_fresh_chassis_command = fresh_chassis_command;

    const auto now = std::chrono::steady_clock::now();
    if (now - diagnostic_time >= std::chrono::seconds(1)) {
      const auto elapsed = std::chrono::duration<double>(now - diagnostic_time).count();
      const auto received_bytes = received_bytes_.load(std::memory_order_relaxed);
      const auto valid_headers = valid_headers_.load(std::memory_order_relaxed);
      const auto imu_frames = valid_imu_frames_.load(std::memory_order_relaxed);
      const auto tx_frames = transmitted_frames_.load(std::memory_order_relaxed);
      const auto crc8_errors = crc8_errors_.load(std::memory_order_relaxed);
      const auto crc16_errors = crc16_errors_.load(std::memory_order_relaxed);
      const auto tx_errors = transmit_errors_.load(std::memory_order_relaxed);
      long long imu_age_ms = -1;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (have_imu_) {
          imu_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_imu_time_).count();
        }
      }
      tools::logger()->info(
        "[SerialRX] connected={} generation={} open_count={} bytes_total={} bytes_per_sec={:.0f} "
        "headers_total={} frames_per_sec={:.1f} imu_total={} imu_hz={:.1f} imu_age_ms={} "
        "fresh_imu={} timeouts={} bad_escape={} crc8_errors={} (+{}) crc16_errors={} (+{}) "
        "invalid_imu={} joint_total={} invalid_joint={} bad_length={} unknown_id={}",
        connected_.load(), connection_generation_.load(), reconnects_.load(), received_bytes,
        (received_bytes - previous_received_bytes) / elapsed, valid_headers,
        (valid_headers - previous_valid_headers) / elapsed, imu_frames,
        (imu_frames - previous_imu_frames) / elapsed, imu_age_ms, fresh_imu,
        receive_timeouts_.load(), invalid_escape_sequences_.load(), crc8_errors,
        crc8_errors - previous_crc8_errors, crc16_errors, crc16_errors - previous_crc16_errors,
        invalid_imu_frames_.load(), valid_joint_frames_.load(), invalid_joint_frames_.load(),
        unexpected_length_frames_.load(), unknown_id_frames_.load());
      const auto raw_bytes = transmitted_raw_bytes_.load(std::memory_order_relaxed);
      const auto wire_bytes = transmitted_wire_bytes_.load(std::memory_order_relaxed);
      const bool tx_tracking = packet.tracking;
      const float tx_yaw = packet.gimbal_yaw;
      const float tx_pitch = packet.gimbal_pitch;
      const float tx_vx = packet.vx;
      const float tx_vy = packet.vy;
      const float tx_wz = packet.wz;
      tools::logger()->info(
        "[SerialTX] frames_total={} frames_per_sec={:.1f} raw_bytes={} wire_bytes={} "
        "escaped_extra={} errors={} (+{}) fresh_self_aim={} fresh_chassis={} tracking={} "
        "yaw_wire={:.4f} pitch_wire={:.4f} imu_yaw={:.4f} yaw_ref={:.4f} "
        "chassis=({:.3f}, {:.3f}, {:.3f}) fire=0 fric_on=0",
        tx_frames, (tx_frames - previous_transmitted_frames) / elapsed, raw_bytes, wire_bytes,
        wire_bytes >= raw_bytes ? wire_bytes - raw_bytes : 0,
        tx_errors, tx_errors - previous_transmit_errors, fresh_command, fresh_chassis_command,
        tx_tracking, tx_yaw, tx_pitch, current_imu_yaw, yaw_reference, tx_vx, tx_vy, tx_wz);
      previous_received_bytes = received_bytes;
      previous_valid_headers = valid_headers;
      previous_imu_frames = imu_frames;
      previous_transmitted_frames = tx_frames;
      previous_crc8_errors = crc8_errors;
      previous_crc16_errors = crc16_errors;
      previous_transmit_errors = tx_errors;
      diagnostic_time = now;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

}  // namespace io
