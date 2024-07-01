cmake_minimum_required(VERSION 3.8)
project(auto_permission)

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
find_package(auto_permission_msgs REQUIRED)
find_package(tier4_debug_msgs REQUIRED)
find_package(wada_vmc_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(report_selector_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tier4_autoware_utils REQUIRED)

add_executable(auto_permission src/auto_permission.cpp)
ament_target_dependencies(auto_permission
  rclcpp
  std_msgs
  geometry_msgs
  auto_permission_msgs
  tier4_debug_msgs
  wada_vmc_msgs
  rviz_2d_overlay_msgs
  autoware_auto_planning_msgs
  report_selector_msgs
  tf2
  nav_msgs
  tier4_autoware_utils
)

install(TARGETS
  auto_permission
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
