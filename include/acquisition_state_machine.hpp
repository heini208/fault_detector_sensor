#pragma once

#include <cstdint>

namespace fault_detector_sensor {

enum class AcquisitionState : std::uint8_t {
  kIdle,
  kRecording,
};

struct TransitionResult {
  bool success;
  bool changed;
  AcquisitionState state;
  const char *detail;
};

class AcquisitionStateMachine {
public:
  AcquisitionState state() const;
  bool is_recording() const;
  TransitionResult set_recording(bool enabled);

private:
  AcquisitionState state_{AcquisitionState::kIdle};
};

const char *ToString(AcquisitionState state);

} // namespace fault_detector_sensor
