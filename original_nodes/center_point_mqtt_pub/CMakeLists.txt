cmake_minimum_required(VERSION 3.8)
project(center_point_mqtt_pub)

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
find_package(autoware_auto_perception_msgs REQUIRED)

add_executable(center_point_mqtt_pub src/center_point_mqtt_pub_core.cpp)
target_link_libraries(center_point_mqtt_pub -lmosquitto)
ament_target_dependencies(center_point_mqtt_pub rclcpp std_msgs geometry_msgs autoware_auto_perception_msgs)

install(TARGETS
  center_point_mqtt_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
