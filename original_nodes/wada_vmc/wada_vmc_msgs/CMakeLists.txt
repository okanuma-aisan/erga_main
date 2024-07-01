cmake_minimum_required(VERSION 3.5)
project(wada_vmc_msgs)

find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
ament_auto_find_build_dependencies()

set(msg_files
  "msg/CanBuffer.idl"
  "msg/Can100_20210103.idl"
  "msg/Can501_20210103.idl"
  "msg/Can502_20210103.idl"
  "msg/Can503_20210103.idl"
  "msg/Can100_20221111.idl"
  "msg/Can501_20221111.idl"
  "msg/Can502_20221111.idl"
  "msg/MqttJoyboardSub.msg"
  "msg/SteerCmd.idl"
  "srv/Blinker.idl"
  "srv/ActualInput.idl"
  "srv/ActualInputUnsigned.idl"
  "srv/ActualInputFloat.idl"
  "srv/DOInput.idl"
)

set(msg_dependencies
  std_msgs
)
  
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES ${msg_dependencies}
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
