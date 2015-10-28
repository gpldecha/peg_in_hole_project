cmake_minimum_required(VERSION 2.8.3)
project(peg_hole_kuka)

find_package(catkin REQUIRED std_tools tf roscpp mathlib robotlib cds_execution
tf cmake_modules kuka_action_server kuka_action_client message_generation peg_filter exploration_planner)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")


add_service_files(
   FILES
   String_cmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)



## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
   CATKIN_DEPENDS std_tools tf
)


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(include ${catkin_INCLUDE_DIRS} ${INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


# --- Create Action Server Node Pour action

add_executable(peg_kuka_server_node  ${SRC_DIR}/action_server_node.cpp)
target_link_libraries(peg_kuka_server_node ${catkin_LIBRARIES})

# --- Create Action Client Node Pour action

add_executable(peg_kuka_client_node  ${INCLUDE_DIR}/peg_hole_kuka/action_client.h
                                     ${SRC_DIR}/cmd_interface.cpp
                                     ${SRC_DIR}/action_client.cpp
                                     ${SRC_DIR}/action_client_node.cpp)

target_link_libraries(peg_kuka_client_node ${catkin_LIBRARIES})


add_executable(peg_cmd_interface_node ${SRC_DIR}/cmd_interface_node.cpp
                                      ${SRC_DIR}/cmd_interface.cpp
                                      ${INCLUDE_DIR}/peg_hole_kuka/cmd_interface.h
)

target_link_libraries(peg_cmd_interface_node ${catkin_LIBRARIES})




