cmake_minimum_required(VERSION 3.8)
project(gmps_driver)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gmps_msgs REQUIRED)                      # CHANGE

add_executable(gmps_tcp_dummy_publisher
 src/gmps_tcp_dummy_publisher.cpp
)
ament_target_dependencies(gmps_tcp_dummy_publisher rclcpp gmps_msgs)

add_executable(gmps_tcp_receiver
 src/gmps_tcp_receiver.cpp
)
ament_target_dependencies(gmps_tcp_receiver rclcpp gmps_msgs std_msgs)

add_executable(gmps_udp_pub
 src/gmps_udp_pub.cpp
)
ament_target_dependencies(gmps_udp_pub rclcpp gmps_msgs std_msgs geometry_msgs)

install(TARGETS
  gmps_tcp_dummy_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  gmps_tcp_receiver
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  gmps_udp_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
