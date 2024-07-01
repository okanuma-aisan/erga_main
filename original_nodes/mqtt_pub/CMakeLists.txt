cmake_minimum_required(VERSION 3.8)
project(mqtt_pub)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mqtt_pub src/mqtt_pub_core.cpp)
target_link_libraries(mqtt_pub -lmosquitto)
ament_target_dependencies(mqtt_pub rclcpp std_msgs)

install(TARGETS
  mqtt_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
