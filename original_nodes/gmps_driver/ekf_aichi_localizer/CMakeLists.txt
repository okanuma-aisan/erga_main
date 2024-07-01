cmake_minimum_required(VERSION 3.8)
project(ekf_aichi_localizer)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(wada_vmc_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gmps_msgs REQUIRED)
find_package(tier4_debug_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)

add_executable(ekf_aichi_localizer
 src/ekf_aichi_localizer.cpp
)

ament_target_dependencies(ekf_aichi_localizer
    rclcpp
    geometry_msgs
    wada_vmc_msgs
    sensor_msgs
    tf2
    tf2_ros
    gmps_msgs
    tier4_debug_msgs
    nmea_msgs
    nav_msgs
    autoware_auto_vehicle_msgs
)

install(TARGETS
ekf_aichi_localizer
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
