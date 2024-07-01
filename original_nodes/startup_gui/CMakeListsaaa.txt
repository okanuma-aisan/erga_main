cmake_minimum_required(VERSION 2.8.3)
project(startup_gui)

# ビルドディレクトリを include ディレクトリに追加するオプションを有効にする。
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# MOC が必要な場合は自動的に実行するオプションを有効にする。
set(CMAKE_AUTOMOC ON)

# UI 定義ファイルの変換が必要な場合は自動的に実行するオプションを有効にする。
set(CMAKE_AUTOUIC ON)

# QtWidgets ライブラリを追加する。
find_package(Qt5Widgets REQUIRED)

#find_package(autoware_build_flags REQUIRED)

#find_package(autoware_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
)

#include_directories(
#)

add_executable(startup_gui src/main.cpp src/mainwindow.cpp src/mainwindow.ui)
target_link_libraries(startup_gui ${catkin_LIBRARIES} Qt5::Widgets)
add_dependencies(startup_gui
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS startup_gui
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(FILES
#	launch/starup_gui.launch
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
