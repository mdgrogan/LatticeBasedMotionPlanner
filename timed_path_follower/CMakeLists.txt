cmake_minimum_required(VERSION 2.8.3)
project(timed_path_follower)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  tf
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
    src/path_follower.cpp
)


target_link_libraries(
  ${PROJECT_NAME} 
    ${catkin_LIBRARIES}
)
