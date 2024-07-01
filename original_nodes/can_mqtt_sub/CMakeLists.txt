cmake_minimum_required(VERSION 3.8)
project(can_mqtt_sub)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(can_mqtt_sub src/can_mqtt_sub_core.cpp)
target_link_libraries(can_mqtt_sub -lmosquitto)
ament_target_dependencies(can_mqtt_sub rclcpp std_msgs geometry_msgs)

install(TARGETS
  can_mqtt_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
