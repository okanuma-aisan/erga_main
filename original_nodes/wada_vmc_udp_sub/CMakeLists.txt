cmake_minimum_required(VERSION 3.8)
project(wada_vmc_udp_sub)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wada_vmc_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)

add_executable(wada_vmc_udp_sub src/wada_vmc_udp_sub.cpp)
#target_link_libraries(ros2_detect_mqtt_pub -lmosquitto)
ament_target_dependencies(wada_vmc_udp_sub rclcpp std_msgs wada_vmc_msgs autoware_auto_vehicle_msgs)

install(TARGETS
  wada_vmc_udp_sub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
