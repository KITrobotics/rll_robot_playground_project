cmake_minimum_required(VERSION 2.8.3)
project(rll_robot_playground_project)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  rll_move
  rll_move_client
  rll_msgs
)

catkin_package(
   CATKIN_DEPENDS rll_move_client rll_msgs
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(playground_iface src/playground_iface.cpp src/playground_iface_node.cpp)
target_link_libraries(playground_iface ${catkin_LIBRARIES})

add_executable(playground_hello_world src/hello_world.cpp)
target_link_libraries(playground_hello_world ${catkin_LIBRARIES})

add_executable(playground src/playground.cpp)
target_link_libraries(playground ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/hello_world.py scripts/playground.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS playground playground_hello_world playground_iface
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
