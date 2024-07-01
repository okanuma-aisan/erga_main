cmake_minimum_required(VERSION 3.8)
project(all_traffic_light_signals_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(traffic_light_signal_msgs REQUIRED)
find_package(autoware_auto_mapping_msgs REQUIRED)
find_package(lanelet2_extension REQUIRED)

include_directories(
  SYSTEM
)

add_executable(all_traffic_light_signals_publisher
  src/all_traffic_light_signals_publisher.cpp
)

ament_target_dependencies(all_traffic_light_signals_publisher
  rclcpp
  traffic_light_signal_msgs
  autoware_auto_mapping_msgs
  lanelet2_extension
)

install(TARGETS
  TARGETS all_traffic_light_signals_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
