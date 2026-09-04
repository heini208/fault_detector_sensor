cmake_minimum_required(VERSION 3.8)
project(fault_detector_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetSensorAcquisition.srv"
)

ament_package()
