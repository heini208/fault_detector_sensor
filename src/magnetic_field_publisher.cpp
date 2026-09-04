#include "magnetic_field_publisher.hpp"

#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

#include "magnetic_field_units.hpp"

namespace fault_detector_sensor {
namespace {

constexpr const char *kNodeName = "fault_detector_sensor";
constexpr int kTimeSynchronizationTimeoutMs = 1000;

} // namespace

MagneticFieldPublisher::MagneticFieldPublisher()
    : node_(rcl_get_zero_initialized_node()),
      publisher_(rcl_get_zero_initialized_publisher()) {}

bool MagneticFieldPublisher::Begin() {
  if (ready_) {
    return true;
  }

  End();
  allocator_ = rcl_get_default_allocator();

  if (!sensor_msgs__msg__MagneticField__init(&message_)) {
    last_error_ = "MagneticField message initialization failed";
    End();
    return false;
  }
  message_initialized_ = true;

  if (!rosidl_runtime_c__String__assign(&message_.header.frame_id,
                                        kMagneticFieldFrame)) {
    last_error_ = "frame ID allocation failed";
    End();
    return false;
  }

  if (rclc_support_init(&support_, 0, nullptr, &allocator_) != RCL_RET_OK) {
    last_error_ = "rcl support initialization failed";
    rcl_reset_error();
    End();
    return false;
  }
  support_initialized_ = true;

  if (rclc_node_init_default(&node_, kNodeName, "", &support_) !=
      RCL_RET_OK) {
    last_error_ = "ROS node creation failed";
    rcl_reset_error();
    End();
    return false;
  }
  node_initialized_ = true;

  if (rclc_publisher_init_best_effort(
          &publisher_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
          kMagneticFieldTopic) != RCL_RET_OK) {
    last_error_ = "MagneticField publisher creation failed";
    rcl_reset_error();
    End();
    return false;
  }
  publisher_initialized_ = true;

  if (rmw_uros_sync_session(kTimeSynchronizationTimeoutMs) != RMW_RET_OK ||
      !rmw_uros_epoch_synchronized()) {
    last_error_ = "Agent time synchronization failed";
    End();
    return false;
  }

  ready_ = true;
  last_error_ = "ready";
  return true;
}

void MagneticFieldPublisher::End() {
  ready_ = false;

  if (support_initialized_) {
    rmw_context_t *context = rcl_context_get_rmw_context(&support_.context);
    if (context != nullptr) {
      (void)rmw_uros_set_context_entity_destroy_session_timeout(context, 0);
    }
  }

  if (publisher_initialized_ && node_initialized_) {
    const rcl_ret_t publisher_fini_result =
        rcl_publisher_fini(&publisher_, &node_);
    (void)publisher_fini_result;
  }
  if (node_initialized_) {
    const rcl_ret_t node_fini_result = rcl_node_fini(&node_);
    (void)node_fini_result;
  }
  if (support_initialized_) {
    (void)rclc_support_fini(&support_);
  }
  if (message_initialized_) {
    sensor_msgs__msg__MagneticField__fini(&message_);
  }

  ResetHandles();
}

bool MagneticFieldPublisher::Publish(std::int16_t x_microtesla,
                                     std::int16_t y_microtesla,
                                     std::int16_t z_microtesla) {
  if (!ready_) {
    last_error_ = "publisher not ready";
    return false;
  }

  const std::int64_t epoch_nanoseconds = rmw_uros_epoch_nanos();
  if (epoch_nanoseconds <= 0) {
    last_error_ = "synchronized timestamp unavailable";
    return false;
  }
  const RosTimestamp timestamp = SplitEpochNanoseconds(epoch_nanoseconds);
  message_.header.stamp.sec = timestamp.seconds;
  message_.header.stamp.nanosec = timestamp.nanoseconds;
  message_.magnetic_field.x = MicroteslaToTesla(x_microtesla);
  message_.magnetic_field.y = MicroteslaToTesla(y_microtesla);
  message_.magnetic_field.z = MicroteslaToTesla(z_microtesla);

  if (rcl_publish(&publisher_, &message_, nullptr) != RCL_RET_OK) {
    last_error_ = "MagneticField publication failed";
    rcl_reset_error();
    return false;
  }

  last_error_ = "ready";
  return true;
}

bool MagneticFieldPublisher::ready() const { return ready_; }

const char *MagneticFieldPublisher::last_error() const { return last_error_; }

void MagneticFieldPublisher::ResetHandles() {
  support_ = {};
  node_ = rcl_get_zero_initialized_node();
  publisher_ = rcl_get_zero_initialized_publisher();
  message_ = {};
  support_initialized_ = false;
  node_initialized_ = false;
  publisher_initialized_ = false;
  message_initialized_ = false;
}

} // namespace fault_detector_sensor
