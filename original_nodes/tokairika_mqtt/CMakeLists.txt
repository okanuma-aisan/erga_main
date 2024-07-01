cmake_minimum_required(VERSION 3.8)
project(tokairika_mqtt)

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
find_package(orig_system_msgs REQUIRED)
find_package(wada_vmc_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(tokairika_mqtt_pub_plane src/tokairika_mqtt_pub_plane.cpp)
target_link_libraries(tokairika_mqtt_pub_plane -lmosquitto)
ament_target_dependencies(tokairika_mqtt_pub_plane rclcpp std_msgs geometry_msgs orig_system_msgs wada_vmc_msgs
                                             autoware_auto_vehicle_msgs tf2)

add_executable(tokairika_mqtt_pub_mgrs src/tokairika_mqtt_pub_mgrs.cpp)
target_link_libraries(tokairika_mqtt_pub_mgrs -lmosquitto -lGeographic)
ament_target_dependencies(tokairika_mqtt_pub_mgrs rclcpp std_msgs geometry_msgs orig_system_msgs wada_vmc_msgs
                                             autoware_auto_vehicle_msgs tf2)

add_executable(tokairika_mqtt_sub src/tokairika_mqtt_sub.cpp)
target_link_libraries(tokairika_mqtt_sub -lmosquitto)
ament_target_dependencies(tokairika_mqtt_sub rclcpp std_msgs geometry_msgs)

install(TARGETS
  tokairika_mqtt_pub_plane
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  tokairika_mqtt_pub_mgrs
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  tokairika_mqtt_sub
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
