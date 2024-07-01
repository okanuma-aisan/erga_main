cmake_minimum_required(VERSION 3.8)
project(pose_twist_to_baselink)

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
find_package(sensor_msgs REQUIRED)
#find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pose_twist_to_baselink src/pose_twist_to_baselink.cpp)
#target_link_libraries(odometry_mqtt_sub -lmosquitto)
#ament_target_dependencies(pose_twist_to_baselink rclcpp std_msgs geometry_msgs message_filters)
ament_target_dependencies(pose_twist_to_baselink rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros)

install(TARGETS
  pose_twist_to_baselink
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
