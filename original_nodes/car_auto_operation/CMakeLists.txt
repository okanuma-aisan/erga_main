cmake_minimum_required(VERSION 3.8)
project(car_auto_operation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wada_vmc_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)

add_executable(car_auto_operation_steer
  src/car_auto_operation_steer.cpp
)

target_compile_definitions(car_auto_operation_steer
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)

#target_link_libraries(mqtt_joyboard_sub -lmosquitto)

ament_target_dependencies(car_auto_operation_steer
  rclcpp
  wada_vmc_msgs
  autoware_auto_control_msgs
)

#ament_export_include_directories(include)

install(TARGETS
  TARGETS car_auto_operation_steer
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
