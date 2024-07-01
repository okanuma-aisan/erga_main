cmake_minimum_required(VERSION 3.8)
project(detect_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mobileye_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(detect_fusion_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(detect_fusion
  src/detect_fusion.cpp
)

ament_target_dependencies(detect_fusion
  rclcpp
  std_msgs
  autoware_auto_perception_msgs
  mobileye_msgs
  detect_fusion_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  nav_msgs
)

install(TARGETS
  TARGETS detect_fusion
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
