cmake_minimum_required(VERSION 3.8)
project(mgrs2latlon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(mgrs2latlon
  src/mgrs2latlon_orig.cpp
)

target_link_libraries(mgrs2latlon -lGeographic)

ament_target_dependencies(mgrs2latlon
  rclcpp
  std_msgs
  nav_msgs
)

install(TARGETS
  TARGETS mgrs2latlon
  DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_package()
