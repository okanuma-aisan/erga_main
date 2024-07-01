cmake_minimum_required(VERSION 3.5)
project(gsm8_interface_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/can/command/B0Command.msg"
  "msg/can/command/B1Command.msg"
  "msg/can/command/M0Command.msg"
  "msg/can/command/M1Command.msg"
  "msg/can/command/M2Command.msg"
  "msg/can/command/data/BLNCommand.msg"
  "msg/can/command/data/BWMCommand.msg"
  "msg/can/command/data/DORCommand.msg"
  "msg/can/command/data/EPBCommand.msg"
  "msg/can/command/data/HRNCommand.msg"
  "msg/can/command/data/LGTCommand.msg"
  "msg/can/command/data/POWCommand.msg"
  "msg/can/command/data/SFTCommand.msg"
  "msg/can/status/B0Status.msg"
  "msg/can/status/B1Status.msg"
  "msg/can/status/M0Status.msg"
  "msg/can/status/M1Status.msg"
  "msg/can/status/M2Status.msg"
  "msg/can/status/M3Status.msg"
  "msg/can/status/M4Status.msg"
  "msg/can/status/M5Status.msg"
  "msg/can/status/data/BLNStatus.msg"
  "msg/can/status/data/BWMStatus.msg"
  "msg/can/status/data/DORStatus.msg"
  "msg/can/status/data/EPBStatus.msg"
  "msg/can/status/data/HRNStatus.msg"
  "msg/can/status/data/LGTStatus.msg"
  "msg/can/status/data/POWStatus.msg"
  "msg/can/status/data/SFTStatus.msg"
  DEPENDENCIES
    "std_msgs"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
