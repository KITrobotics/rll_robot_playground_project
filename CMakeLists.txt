cmake_minimum_required(VERSION 2.8.3)
project(rll_robot_playground_project)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_srvs
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rll_msgs
  rll_move
  genmsg actionlib_msgs actionlib
)

catkin_package(
   CATKIN_DEPENDS rll_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(playground_iface src/playground_iface.cpp)
target_link_libraries(playground_iface ${catkin_LIBRARIES})
