#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

#include "tools/crc.hpp"

namespace
{
constexpr uint8_t kSof = 0x5A;
constexpr uint8_t kEscape = 0x5B;
constexpr float kYawRadToDeg = -57.2957795F;
constexpr float kPitchRadToDeg = 57.2957795F;

struct __attribute__((packed)) HeaderFrame
{
  uint8_t sof;
  uint8_t len;
  uint8_t id;
  uint8_t crc;
};

struct __attribute__((packed)) RobotCommand
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

struct __attribute__((packed)) ImuData
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

std::vector<uint8_t> stuff(const uint8_t * raw, size_t size)
{
  std::vector<uint8_t> wire{raw[0]};
  wire.reserve(size * 2);
  for (size_t i = 1; i < size; ++i) {
    if (raw[i] == kSof) {
      wire.push_back(kEscape);
      wire.push_back(0x01);
    } else if (raw[i] == kEscape) {
      wire.push_back(kEscape);
      wire.push_back(0x02);
    } else {
      wire.push_back(raw[i]);
    }
  }
  return wire;
}

std::vector<uint8_t> unstuff(const std::vector<uint8_t> & wire)
{
  if (wire.empty() || wire.front() != kSof) return {};

  std::vector<uint8_t> raw{wire.front()};
  for (size_t i = 1; i < wire.size(); ++i) {
    if (wire[i] != kEscape) {
      raw.push_back(wire[i]);
      continue;
    }
    if (++i >= wire.size()) return {};
    if (wire[i] == 0x01)
      raw.push_back(kSof);
    else if (wire[i] == 0x02)
      raw.push_back(kEscape);
    else
      return {};
  }
  return raw;
}

bool near(float lhs, float rhs)
{
  return std::abs(lhs - rhs) < 1e-5F;
}

struct LowerCommandState
{
  float vx{0.0F};
  float vy{0.0F};
  float wz{0.0F};
  uint8_t tracking{0};
  uint8_t shoot{0};
  uint32_t last_valid_ms{0};
  bool seen{false};
};

bool applyLowerWatchdog(LowerCommandState & state, uint32_t now_ms)
{
  if (!state.seen || static_cast<uint32_t>(now_ms - state.last_valid_ms) <= 300U) {
    return false;
  }
  state.vx = 0.0F;
  state.vy = 0.0F;
  state.wz = 0.0F;
  state.tracking = 0;
  state.shoot = 0;
  return true;
}
}  // namespace

int main()
{
  static_assert(sizeof(HeaderFrame) == 4);
  static_assert(sizeof(RobotCommand) == 49);
  static_assert(sizeof(ImuData) == 34);
  static_assert(offsetof(RobotCommand, timestamp) == 4);
  static_assert(offsetof(RobotCommand, gimbal_pitch) == 36);
  static_assert(offsetof(RobotCommand, gimbal_yaw) == 40);
  static_assert(offsetof(RobotCommand, fire) == 44);
  static_assert(offsetof(RobotCommand, fric_on) == 45);
  static_assert(offsetof(RobotCommand, tracking) == 46);
  static_assert(offsetof(RobotCommand, crc) == 47);

  RobotCommand command{};
  command.header = {kSof, 43, 0x01, 0};
  command.header.crc = tools::get_crc8(reinterpret_cast<uint8_t *>(&command.header), 3);
  command.timestamp = 0x5B005A00U;
  command.vx = 1.25F;
  command.vy = -0.75F;
  command.wz = 0.4F;
  command.gimbal_pitch = -0.05F * kPitchRadToDeg;
  command.gimbal_yaw = 0.05F * kYawRadToDeg;
  command.tracking = 1;
  command.crc = tools::get_crc16(reinterpret_cast<uint8_t *>(&command), 47);

  if (command.header.crc != 0x7F) return 1;
  if (!tools::check_crc8(reinterpret_cast<uint8_t *>(&command.header), 4)) return 2;
  if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&command), sizeof(command))) return 3;

  const auto wire = stuff(reinterpret_cast<uint8_t *>(&command), sizeof(command));
  if (wire.size() < sizeof(command) + 2) return 4;
  const auto decoded = unstuff(wire);
  if (decoded.size() != sizeof(command)) return 5;
  if (std::memcmp(decoded.data(), &command, sizeof(command)) != 0) return 6;
  if (!tools::check_crc16(decoded.data(), decoded.size())) return 7;

  float lower_pitch = 0.0F;
  float lower_yaw = 0.0F;
  float lower_vx = 0.0F;
  float lower_vy = 0.0F;
  float lower_wz = 0.0F;
  std::memcpy(&lower_vx, decoded.data() + 8, sizeof(float));
  std::memcpy(&lower_vy, decoded.data() + 12, sizeof(float));
  std::memcpy(&lower_wz, decoded.data() + 16, sizeof(float));
  std::memcpy(&lower_pitch, decoded.data() + 36, sizeof(float));
  std::memcpy(&lower_yaw, decoded.data() + 40, sizeof(float));
  if (!near(lower_vx, 1.25F) || !near(lower_vy, -0.75F) || !near(lower_wz, 0.4F)) return 8;
  if (!near(lower_pitch, -2.86478898F)) return 9;
  if (!near(lower_yaw, -2.86478898F)) return 10;
  if (decoded[44] != 0 || decoded[45] != 0 || decoded[46] != 1) return 11;

  RobotCommand safe{};
  safe.header = {kSof, 43, 0x01, 0};
  safe.header.crc = tools::get_crc8(reinterpret_cast<uint8_t *>(&safe.header), 3);
  safe.crc = tools::get_crc16(reinterpret_cast<uint8_t *>(&safe), 47);
  if (safe.crc != 0x188D || !tools::check_crc16(reinterpret_cast<uint8_t *>(&safe), 49)) {
    return 12;
  }

  ImuData imu{};
  imu.header = {kSof, 28, 0x02, 0};
  imu.header.crc = tools::get_crc8(reinterpret_cast<uint8_t *>(&imu.header), 3);
  imu.timestamp = 0x5A5B5A5BU;
  imu.yaw = -0.25F;
  imu.pitch = 0.125F;
  imu.roll = -0.0625F;
  imu.yaw_velocity = 1.0F;
  imu.pitch_velocity = -2.0F;
  imu.roll_velocity = 0.5F;
  imu.crc = tools::get_crc16(reinterpret_cast<uint8_t *>(&imu), sizeof(imu) - 2);
  const auto imu_wire = stuff(reinterpret_cast<uint8_t *>(&imu), sizeof(imu));
  const auto imu_decoded = unstuff(imu_wire);
  if (imu_decoded.size() != sizeof(imu) || std::memcmp(imu_decoded.data(), &imu, sizeof(imu))) {
    return 13;
  }
  if (!tools::check_crc8(imu_decoded.data(), 4) ||
      !tools::check_crc16(imu_decoded.data(), imu_decoded.size())) {
    return 14;
  }

  uint32_t random_state = 0xC0DEC0DEU;
  constexpr size_t kFuzzFrames = 4096;
  for (size_t iteration = 0; iteration < kFuzzFrames; ++iteration) {
    RobotCommand fuzz{};
    auto * bytes = reinterpret_cast<uint8_t *>(&fuzz);
    for (size_t i = sizeof(HeaderFrame); i < sizeof(fuzz) - sizeof(fuzz.crc); ++i) {
      random_state = random_state * 1664525U + 1013904223U;
      bytes[i] = static_cast<uint8_t>(random_state >> 24);
    }
    fuzz.header = {kSof, 43, 0x01, 0};
    fuzz.header.crc = tools::get_crc8(reinterpret_cast<uint8_t *>(&fuzz.header), 3);
    fuzz.fire = 0;
    fuzz.fric_on = 0;
    fuzz.tracking = static_cast<uint8_t>(iteration & 1U);
    fuzz.crc = tools::get_crc16(reinterpret_cast<uint8_t *>(&fuzz), sizeof(fuzz) - 2);

    const auto fuzz_wire = stuff(reinterpret_cast<uint8_t *>(&fuzz), sizeof(fuzz));
    const auto fuzz_decoded = unstuff(fuzz_wire);
    if (fuzz_decoded.size() != sizeof(fuzz) ||
        std::memcmp(fuzz_decoded.data(), &fuzz, sizeof(fuzz))) {
      return 15;
    }
    if (!tools::check_crc8(fuzz_decoded.data(), 4) ||
        !tools::check_crc16(fuzz_decoded.data(), fuzz_decoded.size())) {
      return 16;
    }
  }

  LowerCommandState lower_state{1.25F, -0.75F, 0.4F, 1, 1, 1000U, true};
  if (applyLowerWatchdog(lower_state, 1300U)) return 17;
  if (!applyLowerWatchdog(lower_state, 1301U)) return 18;
  if (
    lower_state.vx != 0.0F || lower_state.vy != 0.0F || lower_state.wz != 0.0F ||
    lower_state.tracking != 0 || lower_state.shoot != 0) {
    return 19;
  }

  LowerCommandState wrap_state{1.0F, 1.0F, 1.0F, 1, 1, UINT32_MAX - 100U, true};
  if (!applyLowerWatchdog(wrap_state, 250U)) return 20;

  const float invalid = std::numeric_limits<float>::quiet_NaN();
  if (std::isfinite(invalid)) return 21;

  std::cout << "PASS frame=49 len=43 header=5A-2B-01-7F wire=" << wire.size()
            << " chassis=(" << lower_vx << "," << lower_vy << "," << lower_wz << ")"
            << " yaw_deg=" << lower_yaw << " pitch_deg=" << lower_pitch
            << " safe_crc=188D imu=34 fuzz=" << kFuzzFrames
            << " watchdog_ms=300 wrap=PASS\n";
  return 0;
}
