cmake_minimum_required(VERSION 2.8.3)
project(peg_hole_policy)

find_package(catkin REQUIRED
roscpp
robot_base_planners
exploration_planner
peg_sensor
message_generation
robot_motion_generation
netft_rdt_driver
particle_filter
optitrack_rviz
dynamic_reconfigure
lwr_ros_interface
lwr_ros_client
)

find_package(Armadillo REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-deprecated-declarations -Wno-reorder")

add_service_files(
   FILES
   String_cmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/gains.cfg
    cfg/cdd_filter.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


include_directories(include ${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})

set(H_FILES ${INCLUDE_DIR}/peg_hole_policy/peg_hole_policy.h
            ${INCLUDE_DIR}/peg_hole_policy/stack_planner/state_machine.h
            ${INCLUDE_DIR}/peg_hole_policy/stack_planner/planning_stack.h

            ${INCLUDE_DIR}/peg_hole_policy/policies/greedy_policy.h
            ${INCLUDE_DIR}/peg_hole_policy/policies/find_table.h
            ${INCLUDE_DIR}/peg_hole_policy/policies/search_policy.h
            ${INCLUDE_DIR}/peg_hole_policy/policies/find_socket.h
            ${INCLUDE_DIR}/peg_hole_policy/policies/go_feature.h
            ${INCLUDE_DIR}/peg_hole_policy/policies/insert_peg.h
            ${INCLUDE_DIR}/peg_hole_policy/policies/base_find.h
            ${INCLUDE_DIR}/peg_hole_policy/policies/force_control.h
            ${INCLUDE_DIR}/peg_hole_policy/policies/gmm_search.h
            ${INCLUDE_DIR}/peg_hole_policy/policies/get_back_on.h
            ${INCLUDE_DIR}/peg_hole_policy/policies/simple_policies.h
            ${INCLUDE_DIR}/peg_hole_policy/policies/specialised_policy.h
)

set(S_FILES ${SRC_DIR}/peg_hole_policy.cpp
            ${SRC_DIR}/stack_planner/planning_stack.cpp
            ${SRC_DIR}/stack_planner/state_machine.cpp
            ${SRC_DIR}/policies/greedy_policy.cpp
            ${SRC_DIR}/policies/specialised_policy.cpp
            ${SRC_DIR}/policies/find_table.cpp
            ${SRC_DIR}/policies/find_socket.cpp
            ${SRC_DIR}/policies/insert_peg.cpp
            ${SRC_DIR}/policies/base_find.cpp
            ${SRC_DIR}/policies/get_back_on.cpp
            ${SRC_DIR}/policies/gmm_search.cpp
            ${SRC_DIR}/policies/search_policy.cpp
            ${SRC_DIR}/policies/go_feature.cpp
            ${SRC_DIR}/policies/force_control.cpp
            ${SRC_DIR}/policies/simple_policies.cpp
)

add_library(${PROJECT_NAME} ${H_FILES} ${S_FILES} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)
