cmake_minimum_required(VERSION 2.8.3)
project(peg_hole_kuka)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED std_tools tf roscpp mathlib robotlib cds_execution tf cmake_modules )

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES state_transformers
   CATKIN_DEPENDS std_tools tf
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(${INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


set(H_FILES  	${INCLUDE_DIR}/search_planner/search_planner.h
                ${INCLUDE_DIR}/search_planner/search_planner_node.h
                ${INCLUDE_DIR}/motion_actions/motion_actions.h
                ${INCLUDE_DIR}/motion_actions/pour_action.h
                ${INCLUDE_DIR}/motion_actions/base_action.h
                ${INCLUDE_DIR}/motion_actions/default_topics.h
                ${INCLUDE_DIR}/action_server/action_server.h
                ${INCLUDE_DIR}/control_interface/control_interface.h
)

set(SRC_FILES 	${SRC_DIR}/search_planner.cpp
                ${SRC_DIR}/search_planner_node.cpp
                ${SRC_DIR}/motion_actions/motion_actions.cpp
                ${SRC_DIR}/motion_actions/pour_action.cpp
                ${SRC_DIR}/motion_actions/base_action.cpp
                ${SRC_DIR}/action_server/action_server.cpp
                ${SRC_DIR}/control_interface/control_interface.cpp
)


add_library(${PROJECT_NAME} SHARED ${H_FILES} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} )

add_executable(pour_action_node ${SRC_DIR}/pour_action_node.cpp )
target_link_libraries(pour_action_node ${PROJECT_NAME})

add_executable(run_search_node ${SRC_DIR}/run_search_node.cpp )
target_link_libraries(run_search_node ${PROJECT_NAME})

