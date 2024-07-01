cmake_minimum_required(VERSION 3.8)
project(novatel_oem7_driver)

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
find_package(sensor_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_sensing_msgs REQUIRED)
find_package(orig_system_msgs REQUIRED)
find_package(tf2)
find_package(autoware_auto_vehicle_msgs)

add_executable(novatel_oem7_driver src/novatel_oem7_driver.cpp)
#target_link_libraries(novatel_oem7_driver)
ament_target_dependencies(novatel_oem7_driver rclcpp std_msgs)

add_executable(novatel_oem7_udp_sub src/novatel_oem7_udp_sub.cpp)
#target_link_libraries(novatel_oem7_driver)
ament_target_dependencies(novatel_oem7_udp_sub rclcpp std_msgs)

add_executable(novatel_oem7_driver_tcp src/novatel_oem7_driver_tcp.cpp)
#target_link_libraries(novatel_oem7_driver_tcp)
ament_target_dependencies(novatel_oem7_driver_tcp rclcpp std_msgs)

add_executable(novatel_sentence_to_navsatfix src/novatel_sentence_to_navsatfix.cpp)
#target_link_libraries(novatel_sentence_to_navsatfix)
ament_target_dependencies(novatel_sentence_to_navsatfix rclcpp std_msgs sensor_msgs geometry_msgs
                          tf2_geometry_msgs autoware_sensing_msgs nmea_msgs orig_system_msgs
                          autoware_auto_vehicle_msgs)

install(TARGETS
  novatel_oem7_driver
  novatel_oem7_udp_sub
  novatel_oem7_driver_tcp
  novatel_sentence_to_navsatfix
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
