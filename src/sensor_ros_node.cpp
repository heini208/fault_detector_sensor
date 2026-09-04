#include "sensor_ros_node.hpp"

#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

#include "magnetic_field_units.hpp"

namespace fault_detector_sensor {
namespace {

constexpr const char *kNodeName = "fault_detector_sensor";
constexpr int kTimeSynchronizationTimeoutMs = 1000;
constexpr std::uint32_t kExecutorSpinTimeoutMs = 5;
constexpr char kExpandedAcquisitionRequestTopic[] =
    "rq/fault_detector/sensors/bmm150_probe/set_acquisitionRequest";

static_assert(sizeof(kExpandedAcquisitionRequestTopic) <=
                  RMW_UXRCE_TOPIC_NAME_MAX_LENGTH,
              "micro-ROS topic-name buffer is too small for the acquisition "
              "service request topic");

} // namespace

SensorRosNode::SensorRosNode(AcquisitionCommandHandler acquisition_handler)
    : acquisition_handler_(acquisition_handler),
      node_(rcl_get_zero_initialized_node()),
      publisher_(rcl_get_zero_initialized_publisher()),
      acquisition_service_(rcl_get_zero_initialized_service()),
      executor_(rclc_executor_get_zero_initialized_executor()) {}

bool SensorRosNode::Begin() {
  if (ready_) {
    return true;
  }

  End();
  allocator_ = rcl_get_default_allocator();

  if (!sensor_msgs__msg__MagneticField__init(&magnetic_field_message_)) {
    last_error_ = "MagneticField message initialization failed";
    End();
    return false;
  }
  magnetic_field_message_initialized_ = true;

  if (!fault_detector_msgs__srv__SetSensorAcquisition_Request__init(
          &acquisition_request_)) {
    last_error_ = "acquisition request initialization failed";
    End();
    return false;
  }
  acquisition_request_initialized_ = true;

  if (!fault_detector_msgs__srv__SetSensorAcquisition_Response__init(
          &acquisition_response_)) {
    last_error_ = "acquisition response initialization failed";
    End();
    return false;
  }
  acquisition_response_initialized_ = true;

  if (!rosidl_runtime_c__String__assign(
          &magnetic_field_message_.header.frame_id, kMagneticFieldFrame)) {
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

  if (rclc_service_init_default(
          &acquisition_service_, &node_,
          ROSIDL_GET_SRV_TYPE_SUPPORT(fault_detector_msgs, srv,
                                      SetSensorAcquisition),
          kSetAcquisitionService) != RCL_RET_OK) {
    last_error_ = "acquisition service creation failed";
    rcl_reset_error();
    End();
    return false;
  }
  service_initialized_ = true;

  if (rclc_executor_init(&executor_, &support_.context, 1, &allocator_) !=
      RCL_RET_OK) {
    last_error_ = "ROS executor initialization failed";
    rcl_reset_error();
    End();
    return false;
  }
  executor_initialized_ = true;

  if (rclc_executor_add_service_with_context(
          &executor_, &acquisition_service_, &acquisition_request_,
          &acquisition_response_, &SensorRosNode::HandleSetAcquisition,
          this) != RCL_RET_OK) {
    last_error_ = "acquisition service registration failed";
    rcl_reset_error();
    End();
    return false;
  }

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

void SensorRosNode::End() {
  ready_ = false;

  if (support_initialized_) {
    rmw_context_t *context = rcl_context_get_rmw_context(&support_.context);
    if (context != nullptr) {
      (void)rmw_uros_set_context_entity_destroy_session_timeout(context, 0);
    }
  }

  if (executor_initialized_) {
    const rcl_ret_t executor_fini_result = rclc_executor_fini(&executor_);
    (void)executor_fini_result;
  }
  if (service_initialized_ && node_initialized_) {
    const rcl_ret_t service_fini_result =
        rcl_service_fini(&acquisition_service_, &node_);
    (void)service_fini_result;
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
  if (acquisition_response_initialized_) {
    fault_detector_msgs__srv__SetSensorAcquisition_Response__fini(
        &acquisition_response_);
  }
  if (acquisition_request_initialized_) {
    fault_detector_msgs__srv__SetSensorAcquisition_Request__fini(
        &acquisition_request_);
  }
  if (magnetic_field_message_initialized_) {
    sensor_msgs__msg__MagneticField__fini(&magnetic_field_message_);
  }

  ResetHandles();
}

void SensorRosNode::Spin() {
  if (!ready_) {
    return;
  }

  const rcl_ret_t result = rclc_executor_spin_some(
      &executor_, RCL_MS_TO_NS(kExecutorSpinTimeoutMs));
  if (result != RCL_RET_OK && result != RCL_RET_TIMEOUT) {
    last_error_ = "ROS executor spin failed";
    rcl_reset_error();
  }
}

bool SensorRosNode::Publish(std::int16_t x_microtesla,
                            std::int16_t y_microtesla,
                            std::int16_t z_microtesla) {
  if (!ready_) {
    last_error_ = "ROS node not ready";
    return false;
  }

  const std::int64_t epoch_nanoseconds = rmw_uros_epoch_nanos();
  if (epoch_nanoseconds <= 0) {
    last_error_ = "synchronized timestamp unavailable";
    return false;
  }
  const RosTimestamp timestamp = SplitEpochNanoseconds(epoch_nanoseconds);
  magnetic_field_message_.header.stamp.sec = timestamp.seconds;
  magnetic_field_message_.header.stamp.nanosec = timestamp.nanoseconds;
  magnetic_field_message_.magnetic_field.x =
      MicroteslaToTesla(x_microtesla);
  magnetic_field_message_.magnetic_field.y =
      MicroteslaToTesla(y_microtesla);
  magnetic_field_message_.magnetic_field.z =
      MicroteslaToTesla(z_microtesla);

  if (rcl_publish(&publisher_, &magnetic_field_message_, nullptr) !=
      RCL_RET_OK) {
    last_error_ = "MagneticField publication failed";
    rcl_reset_error();
    return false;
  }

  last_error_ = "ready";
  return true;
}

bool SensorRosNode::ready() const { return ready_; }

const char *SensorRosNode::last_error() const { return last_error_; }

void SensorRosNode::HandleSetAcquisition(const void *request, void *response,
                                         void *context) {
  const auto *typed_request = static_cast<
      const fault_detector_msgs__srv__SetSensorAcquisition_Request *>(request);
  auto *typed_response = static_cast<
      fault_detector_msgs__srv__SetSensorAcquisition_Response *>(response);
  auto *self = static_cast<SensorRosNode *>(context);

  AcquisitionCommandResult result{false, "acquisition handler unavailable"};
  if (self != nullptr && self->acquisition_handler_ != nullptr) {
    result = self->acquisition_handler_(typed_request->enabled);
  }

  typed_response->success = result.success;
  const char *detail = result.detail != nullptr ? result.detail : "";
  if (!rosidl_runtime_c__String__assign(&typed_response->detail, detail)) {
    typed_response->success = false;
  }
}

void SensorRosNode::ResetHandles() {
  support_ = {};
  node_ = rcl_get_zero_initialized_node();
  publisher_ = rcl_get_zero_initialized_publisher();
  acquisition_service_ = rcl_get_zero_initialized_service();
  executor_ = rclc_executor_get_zero_initialized_executor();
  magnetic_field_message_ = {};
  acquisition_request_ = {};
  acquisition_response_ = {};
  support_initialized_ = false;
  node_initialized_ = false;
  publisher_initialized_ = false;
  service_initialized_ = false;
  executor_initialized_ = false;
  magnetic_field_message_initialized_ = false;
  acquisition_request_initialized_ = false;
  acquisition_response_initialized_ = false;
}

} // namespace fault_detector_sensor
