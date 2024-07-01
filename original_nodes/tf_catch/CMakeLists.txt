cmake_minimum_required(VERSION 3.8)
project(tf_catch)

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
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(tf_catch src/tf_catch.cpp)
#target_link_libraries(tf_catch -lmosquitto)
ament_target_dependencies(tf_catch rclcpp std_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS
  tf_catch
  DESTINATION lib/${PROJECT_NAME})

ament_package()
