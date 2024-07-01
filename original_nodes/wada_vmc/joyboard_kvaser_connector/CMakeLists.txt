cmake_minimum_required(VERSION 3.8)
project(joyboard_kvaser_connector)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fPIC -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(joyboard_kvaser_connector
  src/joyboard_kvaser_connector_core.cpp
)

target_include_directories(joyboard_kvaser_connector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(joyboard_kvaser_connector -lcanlib)
#ament_target_dependencies(joyboard_kvaser_connector canlib)
ament_export_targets(export_joyboard_kvaser_connector HAS_LIBRARY_TARGET)
#ament_export_dependencies(canlib)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS joyboard_kvaser_connector
  EXPORT export_joyboard_kvaser_connector
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
