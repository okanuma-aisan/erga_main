cmake_minimum_required(VERSION 3.8)
project(vehicle_status_mqtt_sub)

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
find_package(autoware_auto_vehicle_msgs REQUIRED)

add_executable(vehicle_status_mqtt_sub src/vehicle_status_mqtt_sub.cpp)
target_link_libraries(vehicle_status_mqtt_sub -lmosquitto)
ament_target_dependencies(vehicle_status_mqtt_sub rclcpp std_msgs autoware_auto_vehicle_msgs)

install(TARGETS
  vehicle_status_mqtt_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
