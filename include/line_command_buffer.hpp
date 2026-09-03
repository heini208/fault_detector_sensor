#pragma once

#include <cstddef>

namespace fault_detector_sensor {

enum class CommandFeedResult {
  kPending,
  kCommandReady,
  kOverflow,
};

class LineCommandBuffer {
public:
  CommandFeedResult Feed(char input);
  const char *command() const;
  void Reset();

private:
  static constexpr std::size_t kCapacity = 32;
  char buffer_[kCapacity]{};
  std::size_t length_{0};
};

} // namespace fault_detector_sensor
