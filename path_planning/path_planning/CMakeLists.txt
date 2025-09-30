cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS cpp/include
  LIBRARIES reeds_shepp_lib
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  cpp/include
  ${catkin_INCLUDE_DIRS}
)

add_library(reeds_shepp_lib
  cpp/src/reeds_shepp.cpp
  cpp/src/theta_star_planner.cpp
  cpp/src/utils.cpp
)

add_executable(demo cpp/demo.cpp)
target_link_libraries(demo
  reeds_shepp_lib
  ${catkin_LIBRARIES}
)
