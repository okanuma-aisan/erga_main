cmake_minimum_required(VERSION 3.5)
project(gsm8_interface)

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

ament_auto_add_library(gsm8_interface_node SHARED
  node/gsm8_interface.cpp
)

rclcpp_components_register_node(gsm8_interface_node
  PLUGIN "GSM8Interface"
  EXECUTABLE gsm8_interface
)

install(PROGRAMS
  scripts/gsm8_accmap.py
  scripts/gsm8_time_analysis.py
  scripts/automatic_door.py
  scripts/acceleration_tester.py
  scripts/acceleration_publisher.py
  scripts/throttle_tester.py
  scripts/throttle_publisher.py
  scripts/brake_pressure_tester.py
  scripts/brake_pressure_publisher.py
  scripts/eps_voltage_tester.py
  scripts/eps_voltage_publisher.py
  scripts/eps_ff_voltage_tester.py
  scripts/eps_ff_voltage_publisher.py
  scripts/yawrate_estimator.py
  scripts/imu_reverse.py
  scripts/validate_map.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_flake8
    ament_cmake_pep257
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    data
    launch
)
