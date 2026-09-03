#include "line_command_buffer.hpp"

namespace fault_detector_sensor {

CommandFeedResult LineCommandBuffer::Feed(char input) {
  if (input == '\r' || input == '\n') {
    if (length_ == 0) {
      return CommandFeedResult::kPending;
    }
    buffer_[length_] = '\0';
    return CommandFeedResult::kCommandReady;
  }

  if (length_ >= kCapacity - 1) {
    Reset();
    return CommandFeedResult::kOverflow;
  }

  buffer_[length_++] = input;
  return CommandFeedResult::kPending;
}

const char *LineCommandBuffer::command() const { return buffer_; }

void LineCommandBuffer::Reset() {
  buffer_[0] = '\0';
  length_ = 0;
}

} // namespace fault_detector_sensor
