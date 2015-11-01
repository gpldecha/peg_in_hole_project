cmake_minimum_required(VERSION 2.8.3)
project(peg_filter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS world_wrapper objects optitrack_rviz tf std_msgs
particle_filter peg_sensor statistics_ml)

find_package(Armadillo)

#-fopenmp

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-unused-variable -fopenmp")        ## Optimize
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

add_service_files(
  FILES
  String_cmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime world_wrapper peg_sensor objects
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


include_directories(${INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${INCLUDE_STATISTICS})

#${INCLUDE_DIR}/peg_filter/peg_sensor/plug_sensor.h
# ${SRC_DIR}/peg_sensor/plug_sensor.cpp
# ${INCLUDE_DIR}/peg_filter/peg_world_wrapper/peg_world_wrapper.h
#               ${SRC_DIR}/peg_world_wrapper/peg_world_wrapper.cpp


set(H_FILES

             ${INCLUDE_DIR}/peg_filter/likelihood.h
             ${INCLUDE_DIR}/peg_filter/motion.h
             ${INCLUDE_DIR}/peg_filter/pf_manager.h
             ${INCLUDE_DIR}/peg_filter/plug_service.h

             ${INCLUDE_DIR}/peg_filter/belief_features/belief_features.h
             ${INCLUDE_DIR}/peg_filter/belief_features/mode_feature.h

)

set(SRC_FILES ${SRC_DIR}/peg_filter/likelihood.cpp
              ${SRC_DIR}/peg_filter/motion.cpp

              ${SRC_DIR}/peg_filter/pf_manager.cpp
              ${SRC_DIR}/peg_filter/plug_service.cpp

              ${SRC_DIR}/belief_features/belief_features.cpp
              ${SRC_DIR}/belief_features/mode_feature.cpp

)


##  SENSOR (LIB) ##

add_library(${PROJECT_NAME} ${H_FILES} ${SRC_FILES}						 )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${LIB_ARMADILLO} ${LIB_STATISTICS} mlpack )


##  PEG FILTER (EXEC) ##

add_executable(${PROJECT_NAME}_node ${SRC_DIR}/peg_filter_node.cpp      	)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}   ${PROJECT_NAME} )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS peg_in_hole peg_in_hole_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_peg_in_hole.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
