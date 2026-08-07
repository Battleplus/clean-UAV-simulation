cmake_minimum_required(VERSION 3.8)
project(px4_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB PX4_MESSAGE_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "msg/*.msg"
  "msg/versioned/*.msg"
)
file(GLOB PX4_SERVICE_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "srv/*.srv"
  "srv/versioned/*.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${PX4_MESSAGE_FILES}
  ${PX4_SERVICE_FILES}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
