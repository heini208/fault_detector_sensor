#pragma once

#include <cstdint>

#include <fault_detector_msgs/srv/set_sensor_acquisition.h>
#include <rcl/rcl.h>
#include <rcl/service.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/magnetic_field.h>

namespace fault_detector_sensor {

struct AcquisitionCommandResult {
  bool success;
  const char *detail;
};

using AcquisitionCommandHandler = AcquisitionCommandResult (*)(bool enabled);

class SensorRosNode {
public:
  explicit SensorRosNode(AcquisitionCommandHandler acquisition_handler);

  bool Begin();
  void End();
  void Spin();
  bool Publish(std::int16_t x_microtesla, std::int16_t y_microtesla,
               std::int16_t z_microtesla);

  bool ready() const;
  const char *last_error() const;

private:
  static void HandleSetAcquisition(const void *request, void *response,
                                   void *context);
  void ResetHandles();

  AcquisitionCommandHandler acquisition_handler_;
  rcl_allocator_t allocator_{};
  rclc_support_t support_{};
  rcl_node_t node_;
  rcl_publisher_t publisher_;
  rcl_service_t acquisition_service_;
  rclc_executor_t executor_;
  sensor_msgs__msg__MagneticField magnetic_field_message_{};
  fault_detector_msgs__srv__SetSensorAcquisition_Request
      acquisition_request_{};
  fault_detector_msgs__srv__SetSensorAcquisition_Response
      acquisition_response_{};

  bool support_initialized_{false};
  bool node_initialized_{false};
  bool publisher_initialized_{false};
  bool service_initialized_{false};
  bool executor_initialized_{false};
  bool magnetic_field_message_initialized_{false};
  bool acquisition_request_initialized_{false};
  bool acquisition_response_initialized_{false};
  bool ready_{false};
  const char *last_error_{"not initialized"};
};

constexpr const char *kMagneticFieldTopic =
    "/sensors/bmm150_probe/magnetic_field";
constexpr const char *kMagneticFieldFrame = "bmm150_probe_probe";
constexpr const char *kSetAcquisitionService =
    "/fault_detector/sensors/bmm150_probe/set_acquisition";

} // namespace fault_detector_sensor
