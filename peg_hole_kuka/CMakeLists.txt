cmake_minimum_required(VERSION 2.8.3)
project(peg_hole_kuka)

find_package(catkin REQUIRED
tf
roscpp
cmake_modules
message_generation
peg_filter
exploration_planner
robot_base_planners
peg_hole_policy
simple_example
netft_rdt_driver
)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-deprecated-declarations")


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
   CATKIN_DEPENDS tf simple_example
)


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(include ${catkin_INCLUDE_DIRS} ${INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


add_library(cmd_interface  ${INCLUDE_DIR}/peg_hole_kuka/cmd_interface.h ${SRC_DIR}/cmd_interface.cpp)
target_link_libraries(cmd_interface ${catkin_LIBRARIES})


# --- Create Action Server Node Pour action

add_executable(peg_kuka_action_node  ${SRC_DIR}/client_action_node.cpp)
target_link_libraries(peg_kuka_action_node ${catkin_LIBRARIES})

# --- Create Action Client Node Pour action

add_executable(peg_kuka_client_node  ${INCLUDE_DIR}/peg_hole_kuka/client.h

                                     ${SRC_DIR}/client.cpp
                                     ${SRC_DIR}/client_node.cpp)

target_link_libraries(peg_kuka_client_node ${catkin_LIBRARIES})


add_executable(peg_cmd_interface_node ${SRC_DIR}/cmd_interface_node.cpp
                                      ${SRC_DIR}/cmd_interface.cpp
                                      ${SRC_DIR}/console.cpp
                                      ${INCLUDE_DIR}/peg_hole_kuka/console.h
                                      ${INCLUDE_DIR}/peg_hole_kuka/cmd_interface.h
)

target_link_libraries(peg_cmd_interface_node ${catkin_LIBRARIES})




