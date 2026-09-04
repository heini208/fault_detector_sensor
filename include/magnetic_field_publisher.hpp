#pragma once

#include <cstdint>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/magnetic_field.h>

namespace fault_detector_sensor {

class MagneticFieldPublisher {
public:
  MagneticFieldPublisher();

  bool Begin();
  void End();
  bool Publish(std::int16_t x_microtesla, std::int16_t y_microtesla,
               std::int16_t z_microtesla);

  bool ready() const;
  const char *last_error() const;

private:
  void ResetHandles();

  rcl_allocator_t allocator_{};
  rclc_support_t support_{};
  rcl_node_t node_;
  rcl_publisher_t publisher_;
  sensor_msgs__msg__MagneticField message_{};

  bool support_initialized_{false};
  bool node_initialized_{false};
  bool publisher_initialized_{false};
  bool message_initialized_{false};
  bool ready_{false};
  const char *last_error_{"not initialized"};
};

constexpr const char *kMagneticFieldTopic =
    "/sensors/bmm150_probe/magnetic_field";
constexpr const char *kMagneticFieldFrame = "bmm150_probe_probe";

} // namespace fault_detector_sensor
