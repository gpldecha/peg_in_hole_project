cmake_minimum_required(VERSION 2.8.3)
project(socket_table_broadcaster)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  optitrack_rviz
  roscpp
  tf
  geometry_msgs
  message_runtime
)

add_service_files(
  FILES
  String_cmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES socket_table_broadcaster
   CATKIN_DEPENDS optitrack_rviz roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-deprecated-declarations")        ## Optimize
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(HEADER_FILES ${INCLUDE_DIR}/socket_table_broadcaster/save.h
                 ${INCLUDE_DIR}/socket_table_broadcaster/load.h
                 ${INCLUDE_DIR}/socket_table_broadcaster/services.h)

set(SRC_FILES  ${SRC_DIR}/save.cpp
               ${SRC_DIR}/load.cpp
               ${SRC_DIR}/services.cpp
               ${SRC_DIR}/run.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


add_dependencies(${PROJECT_NAME}  ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
