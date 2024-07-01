cmake_minimum_required(VERSION 3.8)
project(transform_center_velocity)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(transform_center_velocity src/transform_center_velocity.cpp)
target_link_libraries(transform_center_velocity ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
include_directories(${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
ament_target_dependencies(transform_center_velocity rclcpp tf2 tf2_ros geometry_msgs sensor_msgs
                                                    tf2_geometry_msgs)

install(TARGETS
  transform_center_velocity
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
