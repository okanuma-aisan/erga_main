cmake_minimum_required(VERSION 3.8)
project(autoware_control_cmd_mqtt_send)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)

add_executable(autoware_control_cmd_mqtt_send src/autoware_control_cmd_mqtt_send.cpp)
target_link_libraries(autoware_control_cmd_mqtt_send -lmosquitto)
ament_target_dependencies(autoware_control_cmd_mqtt_send rclcpp std_msgs geometry_msgs autoware_auto_control_msgs)

install(TARGETS
  autoware_control_cmd_mqtt_send
  DESTINATION lib/${PROJECT_NAME})

ament_package()
