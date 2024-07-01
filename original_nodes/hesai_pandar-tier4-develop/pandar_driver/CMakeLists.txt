cmake_minimum_required(VERSION 3.5)
project(pandar_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
)

ament_auto_add_library(pandar_driver SHARED
  src/driver/pandar_driver.cpp
  src/driver/pandar_driver_core.cpp
  src/lib/socket_input.cpp
  src/lib/pcap_input.cpp
)
target_link_libraries(pandar_driver pcap)


rclcpp_components_register_node(pandar_driver
  PLUGIN "pandar_driver::PandarDriver"
  EXECUTABLE pandar_driver_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
