cmake_minimum_required(VERSION 3.8)
project(math_slip_angle)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(math_slip_angle src/math_slip_angle.cpp)
#target_link_libraries(tf_catch -lmosquitto)
ament_target_dependencies(math_slip_angle
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  math_slip_angle
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

#install(DIRECTORY
#  config
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_package()
