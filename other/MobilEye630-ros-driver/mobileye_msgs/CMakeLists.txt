cmake_minimum_required(VERSION 3.5)
project(mobileye_msgs)

find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
ament_auto_find_build_dependencies()

set(msg_files
"msg/LKAlane.msg"
"msg/ReferencePoints.msg"
"msg/LKA.msg"
"msg/Lane.msg"
"msg/ObstacleStatus.msg"
"msg/ObstacleData.msg"
"msg/CarInfo.msg"
"msg/Gyro.msg"
"msg/SystemWarnings.msg"
"msg/TSR.msg"
"msg/TSRVisionOnlySign.msg"
"msg/ExtLogData.msg"
"msg/SmartADAS.msg"
"msg/SeeQ.msg"
"msg/StandardCAN.msg"
"msg/MobileyeInfo.msg"
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