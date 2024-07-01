cmake_minimum_required(VERSION 3.8)
project(pose_udp_pub)

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
find_package(nav_msgs REQUIRED)
find_package(tier4_debug_msgs REQUIRED)

add_executable(pose_udp_pub src/pose_udp_pub.cpp)
#target_link_libraries(ros2_detect_mqtt_pub -lmosquitto)
ament_target_dependencies(pose_udp_pub rclcpp std_msgs geometry_msgs tier4_debug_msgs nav_msgs)

install(TARGETS
  pose_udp_pub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
