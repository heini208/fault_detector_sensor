#pragma once

#include <cstdint>

namespace fault_detector_sensor {
namespace hardware {

constexpr std::int8_t kTftChipSelectPin = 5;
constexpr std::int8_t kTftResetPin = 16;
constexpr std::int8_t kTftDataCommandPin = 19;
constexpr std::int8_t kTftMosiPin = 23;
constexpr std::int8_t kTftClockPin = 18;

constexpr std::uint32_t kSerialBaud = 115200;
constexpr std::uint32_t kSamplePeriodMs = 100;
constexpr std::uint32_t kCalibrationDurationMs = 10000;
constexpr std::uint32_t kRosEntityRetryPeriodMs = 2000;

} // namespace hardware
} // namespace fault_detector_sensor
