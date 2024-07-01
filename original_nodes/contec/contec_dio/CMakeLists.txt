cmake_minimum_required(VERSION 3.8)
project(contec_dio)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(wada_vmc_msgs REQUIRED)
find_package(auto_permission_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

include_directories(
  include
  ${wada_vmc_INCLUDE_DIRS}
)

add_library(contec_dio SHARED
  src/contec_dio.cpp
)

rclcpp_components_register_nodes(contec_dio "contec::ContecDio")

#target_compile_definitions(kvaser_receiver
#  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
#)

target_compile_options(contec_dio PUBLIC -Wall -Wextra -Wpedantic -fpermissive)

target_link_libraries(contec_dio -lcdio)

ament_target_dependencies(contec_dio
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  autoware_auto_vehicle_msgs
  autoware_adapi_v1_msgs
  wada_vmc_msgs
  auto_permission_msgs
  autoware_adapi_v1_msgs
)

install(TARGETS
  TARGETS contec_dio
  EXPORT export_contec_dio
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
