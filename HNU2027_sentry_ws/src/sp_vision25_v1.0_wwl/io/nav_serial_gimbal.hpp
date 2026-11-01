#ifndef IO__NAV_SERIAL_GIMBAL_HPP
#define IO__NAV_SERIAL_GIMBAL_HPP

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "io/gimbal/gimbal.hpp"
#include "serial/serial.h"

namespace io
{

class NavSerialGimbal
{
public:
  explicit NavSerialGimbal(const std::string & config_path);
  ~NavSerialGimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  bool imuFresh() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point timestamp);
  Eigen::Quaterniond latestQ() const;

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
    float pitch_vel, float pitch_acc);
  void send(VisionToGimbal command);
  void setChassisCommand(float vx, float vy, float wz);
  bool stopAndWait(std::chrono::milliseconds timeout = std::chrono::milliseconds(250));

private:
  struct ImuSample
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  bool openSerial();
  void disconnectSerial(const std::string & reason, uint64_t expected_generation = 0);
  void clearImuState();
  size_t readSerial(uint8_t * data, size_t size);
  size_t writeSerial(const uint8_t * data, size_t size, uint64_t connection_generation);
  bool readExact(uint8_t * data, size_t size);
  bool readDecodedExact(uint8_t * data, size_t size);
  size_t writeFrame(const uint8_t * data, size_t size, uint64_t connection_generation);
  void receiveLoop();
  void sendLoop();
  void handleFrame(uint8_t id, const std::vector<uint8_t> & frame);

  serial::Serial serial_;
  std::string port_;
  uint32_t baud_{115200U};
  mutable std::shared_mutex serial_lifecycle_mutex_;
  std::atomic<bool> connected_{false};
  std::atomic<uint64_t> connection_generation_{0};
  std::atomic<bool> quit_{false};
  std::atomic<uint64_t> received_bytes_{0};
  std::atomic<uint64_t> receive_timeouts_{0};
  std::atomic<uint64_t> invalid_escape_sequences_{0};
  std::atomic<uint64_t> valid_headers_{0};
  std::atomic<uint64_t> crc8_errors_{0};
  std::atomic<uint64_t> crc16_errors_{0};
  std::atomic<uint64_t> valid_imu_frames_{0};
  std::atomic<uint64_t> valid_joint_frames_{0};
  std::atomic<uint64_t> invalid_imu_frames_{0};
  std::atomic<uint64_t> invalid_joint_frames_{0};
  std::atomic<uint64_t> unexpected_length_frames_{0};
  std::atomic<uint64_t> unknown_id_frames_{0};
  std::atomic<uint64_t> transmitted_frames_{0};
  std::atomic<uint64_t> transmitted_raw_bytes_{0};
  std::atomic<uint64_t> transmitted_wire_bytes_{0};
  std::atomic<uint64_t> transmit_errors_{0};
  std::atomic<uint64_t> reconnects_{0};
  std::thread receive_thread_;
  std::thread send_thread_;

  mutable std::mutex state_mutex_;
  GimbalState state_{0.0F, 0.0F, 0.0F, 0.0F, 22.0F, 0};
  bool have_imu_{false};
  bool yaw_reference_valid_{false};
  float yaw_reference_{0.0F};
  bool have_controller_timestamp_{false};
  uint32_t last_controller_timestamp_{0};
  std::chrono::steady_clock::time_point last_imu_time_{};

  mutable std::mutex imu_mutex_;
  std::condition_variable imu_cv_;
  std::deque<ImuSample> imu_samples_;

  std::mutex command_mutex_;
  float command_yaw_{0.0F};
  float command_pitch_{0.0F};
  bool command_tracking_{false};
  bool have_command_{false};
  uint64_t command_connection_generation_{0};
  std::chrono::steady_clock::time_point last_command_time_{};
  float yaw_command_scale_{-57.2957795F};
  float pitch_command_scale_{57.2957795F};
  bool yaw_relative_to_start_{false};
  bool byte_stuffing_{true};
  bool manual_test_{false};
  float manual_test_yaw_{0.05F};
  float manual_test_pitch_{0.0F};
  std::chrono::milliseconds command_timeout_{200};
  std::mutex chassis_command_mutex_;
  float chassis_vx_{0.0F};
  float chassis_vy_{0.0F};
  float chassis_wz_{0.0F};
  bool have_chassis_command_{false};
  uint64_t chassis_command_connection_generation_{0};
  std::chrono::steady_clock::time_point last_chassis_command_time_{};
  std::chrono::milliseconds chassis_command_timeout_{250};
  std::chrono::milliseconds imu_timeout_{500};
  std::chrono::milliseconds reconnect_interval_{1000};

  std::atomic<uint64_t> safe_request_generation_{0};
  std::atomic<uint64_t> safe_sent_generation_{0};
  std::mutex safe_command_mutex_;
  std::condition_variable safe_command_cv_;
};

}  // namespace io

#endif  // IO__NAV_SERIAL_GIMBAL_HPP
