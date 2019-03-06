cmake_minimum_required(VERSION 2.8.3)
project(lidar_websocket)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(PROGRAMS
  scripts/lidar_client.js
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
