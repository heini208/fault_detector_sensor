#include <unity.h>

#include "acquisition_state_machine.hpp"
#include "line_command_buffer.hpp"
#include "magnetic_field_units.hpp"

using fault_detector_sensor::AcquisitionState;
using fault_detector_sensor::AcquisitionStateMachine;
using fault_detector_sensor::CommandFeedResult;
using fault_detector_sensor::LineCommandBuffer;
using fault_detector_sensor::MicroteslaToTesla;
using fault_detector_sensor::SplitEpochNanoseconds;
using fault_detector_sensor::ToString;

void setUp() {}

void tearDown() {}

void test_starts_idle() {
  AcquisitionStateMachine state_machine;

  TEST_ASSERT_EQUAL_INT(static_cast<int>(AcquisitionState::kIdle),
                        static_cast<int>(state_machine.state()));
  TEST_ASSERT_FALSE(state_machine.is_recording());
}

void test_start_is_acknowledged_and_changes_state() {
  AcquisitionStateMachine state_machine;

  const auto result = state_machine.set_recording(true);

  TEST_ASSERT_TRUE(result.success);
  TEST_ASSERT_TRUE(result.changed);
  TEST_ASSERT_TRUE(state_machine.is_recording());
  TEST_ASSERT_EQUAL_STRING("recording started", result.detail);
}

void test_repeated_start_is_idempotent() {
  AcquisitionStateMachine state_machine;
  state_machine.set_recording(true);

  const auto result = state_machine.set_recording(true);

  TEST_ASSERT_TRUE(result.success);
  TEST_ASSERT_FALSE(result.changed);
  TEST_ASSERT_EQUAL_STRING("already recording", result.detail);
}

void test_stop_returns_to_idle() {
  AcquisitionStateMachine state_machine;
  state_machine.set_recording(true);

  const auto result = state_machine.set_recording(false);

  TEST_ASSERT_TRUE(result.success);
  TEST_ASSERT_TRUE(result.changed);
  TEST_ASSERT_FALSE(state_machine.is_recording());
  TEST_ASSERT_EQUAL_STRING("recording stopped", result.detail);
}

void test_repeated_stop_is_idempotent() {
  AcquisitionStateMachine state_machine;

  const auto result = state_machine.set_recording(false);

  TEST_ASSERT_TRUE(result.success);
  TEST_ASSERT_FALSE(result.changed);
  TEST_ASSERT_EQUAL_STRING("already idle", result.detail);
}

void test_state_names_match_protocol_terms() {
  TEST_ASSERT_EQUAL_STRING("IDLE", ToString(AcquisitionState::kIdle));
  TEST_ASSERT_EQUAL_STRING("RECORDING", ToString(AcquisitionState::kRecording));
}

void test_line_feed_submits_command() {
  LineCommandBuffer buffer;

  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandFeedResult::kPending),
                        static_cast<int>(buffer.Feed('s')));
  buffer.Feed('t');
  buffer.Feed('a');
  buffer.Feed('r');
  buffer.Feed('t');

  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandFeedResult::kCommandReady),
                        static_cast<int>(buffer.Feed('\n')));
  TEST_ASSERT_EQUAL_STRING("start", buffer.command());
}

void test_carriage_return_submits_command() {
  LineCommandBuffer buffer;
  buffer.Feed('s');
  buffer.Feed('t');
  buffer.Feed('o');
  buffer.Feed('p');

  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandFeedResult::kCommandReady),
                        static_cast<int>(buffer.Feed('\r')));
  TEST_ASSERT_EQUAL_STRING("stop", buffer.command());
}

void test_second_character_of_crlf_does_not_submit_empty_command() {
  LineCommandBuffer buffer;
  buffer.Feed('s');
  buffer.Feed('t');
  buffer.Feed('a');
  buffer.Feed('t');
  buffer.Feed('u');
  buffer.Feed('s');
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandFeedResult::kCommandReady),
                        static_cast<int>(buffer.Feed('\r')));
  buffer.Reset();

  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandFeedResult::kPending),
                        static_cast<int>(buffer.Feed('\n')));
}

void test_command_buffer_accepts_network_configuration_command() {
  LineCommandBuffer buffer;
  const char command[] =
      "set-wifi fault-detector-network a-long-test-password";
  for (const char character : command) {
    if (character != '\0') {
      TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandFeedResult::kPending),
                            static_cast<int>(buffer.Feed(character)));
    }
  }

  TEST_ASSERT_EQUAL_INT(static_cast<int>(CommandFeedResult::kCommandReady),
                        static_cast<int>(buffer.Feed('\n')));
  TEST_ASSERT_EQUAL_STRING(command, buffer.command());
}

void test_magnetic_field_units_are_converted_to_tesla() {
  TEST_ASSERT_FLOAT_WITHIN(1.0e-9F, 44.0e-6F,
                           static_cast<float>(MicroteslaToTesla(44)));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-9F, -22.0e-6F,
                           static_cast<float>(MicroteslaToTesla(-22)));
}

void test_epoch_nanoseconds_are_split_for_ros_header() {
  const auto timestamp = SplitEpochNanoseconds(1788435776352449000LL);

  TEST_ASSERT_EQUAL_INT32(1788435776, timestamp.seconds);
  TEST_ASSERT_EQUAL_UINT32(352449000, timestamp.nanoseconds);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_starts_idle);
  RUN_TEST(test_start_is_acknowledged_and_changes_state);
  RUN_TEST(test_repeated_start_is_idempotent);
  RUN_TEST(test_stop_returns_to_idle);
  RUN_TEST(test_repeated_stop_is_idempotent);
  RUN_TEST(test_state_names_match_protocol_terms);
  RUN_TEST(test_line_feed_submits_command);
  RUN_TEST(test_carriage_return_submits_command);
  RUN_TEST(test_second_character_of_crlf_does_not_submit_empty_command);
  RUN_TEST(test_command_buffer_accepts_network_configuration_command);
  RUN_TEST(test_magnetic_field_units_are_converted_to_tesla);
  RUN_TEST(test_epoch_nanoseconds_are_split_for_ros_header);
  return UNITY_END();
}
