cmake_minimum_required(VERSION 3.8)
project(wada_vmc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(joyboard_kvaser_connector REQUIRED)
find_package(wada_vmc_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(auto_permission_msgs REQUIRED)

add_executable(kvaser_receiver
  src/kvaser_receiver/kvaser_receiver.cpp
)
add_executable(kvaser_sender_20210103
  src/kvaser_sender/kvaser_sender_20210103.cpp
)
add_executable(kvaser_sender_20221111
  src/kvaser_sender/kvaser_sender_20221111.cpp
)
add_executable(receive_to_topic_20210103
  src/receive_to_topic/receive_to_topic_20210103.cpp
)
add_executable(receive_to_topic_20221111
  src/receive_to_topic/receive_to_topic_20221111.cpp
)
add_executable(vmc_calculator_20210103
  src/vmc_calculator/vmc_calculator_20210103.cpp
)
add_executable(vmc_calculator_20221111
  src/vmc_calculator/vmc_calculator_20221111.cpp
)
add_executable(mqtt_joyboard_sub
  src/mqtt_joyboard_sub/mqtt_joyboard_sub.cpp
)

target_include_directories(vmc_calculator_20210103
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(vmc_calculator_20221111
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(kvaser_receiver
  PRIVATE "MY_LIBRARY_BUILDING_LIBRARY"
)

target_compile_options(kvaser_receiver PUBLIC -Wall -Wextra -Wpedantic -Werror)
target_compile_options(kvaser_sender_20210103 PUBLIC -Wall -Wextra -Wpedantic -Werror)
target_compile_options(kvaser_sender_20221111 PUBLIC -Wall -Wextra -Wpedantic -Werror)
target_compile_options(receive_to_topic_20210103 PUBLIC -Wall -Wextra -Wpedantic -Werror)
target_compile_options(receive_to_topic_20221111 PUBLIC -Wall -Wextra -Wpedantic -Werror)
target_compile_options(vmc_calculator_20210103 PUBLIC -Wall -Wextra -Wpedantic -Werror)
target_compile_options(vmc_calculator_20221111 PUBLIC -Wall -Wextra -Wpedantic -Werror)
target_compile_options(mqtt_joyboard_sub PUBLIC -Wall -Wextra -Wpedantic -Werror)

target_link_libraries(mqtt_joyboard_sub -lmosquitto)

ament_target_dependencies(kvaser_receiver
  rclcpp
  joyboard_kvaser_connector
  wada_vmc_msgs
)

ament_target_dependencies(kvaser_sender_20210103
  rclcpp
  joyboard_kvaser_connector
  wada_vmc_msgs
)

ament_target_dependencies(kvaser_sender_20221111
  rclcpp
  joyboard_kvaser_connector
  wada_vmc_msgs
)

ament_target_dependencies(receive_to_topic_20210103
  rclcpp
  std_msgs
  wada_vmc_msgs
  autoware_auto_vehicle_msgs
)

ament_target_dependencies(receive_to_topic_20221111
  rclcpp
  std_msgs
  wada_vmc_msgs
  autoware_auto_vehicle_msgs
)

ament_target_dependencies(vmc_calculator_20210103
  rclcpp
  std_msgs
  std_srvs
  wada_vmc_msgs
  autoware_auto_control_msgs
  autoware_auto_vehicle_msgs
  tier4_vehicle_msgs
  autoware_adapi_v1_msgs
  auto_permission_msgs
)

ament_target_dependencies(vmc_calculator_20221111
  rclcpp
  std_msgs
  std_srvs
  wada_vmc_msgs
  autoware_auto_control_msgs
  autoware_auto_vehicle_msgs
  tier4_vehicle_msgs
  autoware_adapi_v1_msgs
  auto_permission_msgs
)

ament_target_dependencies(mqtt_joyboard_sub
  rclcpp
  std_msgs
  wada_vmc_msgs
)

ament_export_include_directories(include)

install(TARGETS
  TARGETS kvaser_receiver
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  TARGETS kvaser_sender_20210103
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  TARGETS kvaser_sender_20221111
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  TARGETS receive_to_topic_20210103
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  TARGETS receive_to_topic_20221111
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  TARGETS vmc_calculator_20210103
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  TARGETS vmc_calculator_20221111
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  TARGETS mqtt_joyboard_sub
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
