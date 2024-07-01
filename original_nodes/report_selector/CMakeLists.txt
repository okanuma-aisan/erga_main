cmake_minimum_required(VERSION 3.8)
project(report_selector)

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
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(report_selector_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)

add_executable(report_selector src/report_selector.cpp)
add_executable(velocity_report_selector src/velocity_report_selector.cpp)

#target_link_libraries(tf_catch -lmosquitto)
ament_target_dependencies(report_selector
  rclcpp
  std_msgs
  autoware_auto_vehicle_msgs
  report_selector_msgs
  nmea_msgs
)

ament_target_dependencies(velocity_report_selector
  rclcpp
  std_msgs
  autoware_auto_vehicle_msgs
  nmea_msgs
)

install(TARGETS
  report_selector
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  velocity_report_selector
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
