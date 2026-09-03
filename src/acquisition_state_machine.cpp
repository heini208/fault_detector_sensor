#include "acquisition_state_machine.hpp"

namespace fault_detector_sensor {

AcquisitionState AcquisitionStateMachine::state() const { return state_; }

bool AcquisitionStateMachine::is_recording() const {
  return state_ == AcquisitionState::kRecording;
}

TransitionResult AcquisitionStateMachine::set_recording(bool enabled) {
  const auto requested_state =
      enabled ? AcquisitionState::kRecording : AcquisitionState::kIdle;

  if (state_ == requested_state) {
    return {
        true,
        false,
        state_,
        enabled ? "already recording" : "already idle",
    };
  }

  state_ = requested_state;
  return {
      true,
      true,
      state_,
      enabled ? "recording started" : "recording stopped",
  };
}

const char *ToString(AcquisitionState state) {
  switch (state) {
  case AcquisitionState::kIdle:
    return "IDLE";
  case AcquisitionState::kRecording:
    return "RECORDING";
  }
  return "UNKNOWN";
}

} // namespace fault_detector_sensor
