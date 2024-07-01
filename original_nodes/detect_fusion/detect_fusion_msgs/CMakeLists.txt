cmake_minimum_required(VERSION 3.5)
project(detect_fusion_msgs)

find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mobileye_msgs REQUIRED)

ament_auto_find_build_dependencies()

set(msg_files
  "msg/MobileyeObstacleStamp.idl"
  "msg/MobileyeObstacleStampArray.idl"
)

set(msg_dependencies
  std_msgs
  mobileye_msgs
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
