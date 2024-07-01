cmake_minimum_required(VERSION 3.8)
project(mgrs_gcs_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tier4_map_msgs REQUIRED)
find_package(tier4_api_msgs REQUIRED)
find_package(tf2 REQUIRED)

# make sure to install libgeographic-dev (ubuntu)
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_executable(conversion_node src/conversion_node.cpp)
ament_target_dependencies(conversion_node rclcpp nav_msgs tier4_map_msgs tier4_api_msgs tf2 GeographicLib)

install(TARGETS
  conversion_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
