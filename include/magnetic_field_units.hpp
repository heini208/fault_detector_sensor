#pragma once

#include <cstdint>

namespace fault_detector_sensor {

struct RosTimestamp {
  std::int32_t seconds;
  std::uint32_t nanoseconds;
};

constexpr double MicroteslaToTesla(std::int16_t value_microtesla) {
  return static_cast<double>(value_microtesla) * 1.0e-6;
}

constexpr RosTimestamp SplitEpochNanoseconds(std::int64_t epoch_nanoseconds) {
  return {
      static_cast<std::int32_t>(epoch_nanoseconds / 1000000000LL),
      static_cast<std::uint32_t>(epoch_nanoseconds % 1000000000LL),
  };
}

} // namespace fault_detector_sensor
