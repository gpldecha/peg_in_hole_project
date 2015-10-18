cmake_minimum_required(VERSION 2.8.3)
project(peg_filter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS world_wrapper objects optitrack_rviz tf std_msgs particle_filter plug_sensor_models)

find_package(Armadillo)

#-fopenmp

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-unused-variable -fopenmp")        ## Optimize
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

set(INCLUDE_STATISTICS /home/guillaume/CppWorkSpace/Statistics/include)
set(LIB_STATISTICS /home/guillaume/CppWorkSpace/Statistics/lib/libstatistics.so)

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
  LIBRARIES peg_sensor
  CATKIN_DEPENDS message_runtime world_wrapper objects plug_sensor_models
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


include_directories(${INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${INCLUDE_STATISTICS})


set(H_FILES  ${INCLUDE_DIR}/peg_sensor/plug_sensor.h 
             ${INCLUDE_DIR}/peg_world_wrapper/plug_world_wrapper.h
             ${INCLUDE_DIR}/peg_filter/plug_likelihood.h
             ${INCLUDE_DIR}/peg_filter/motion_model.h
             ${INCLUDE_DIR}/peg_filter/pf_manager.h
             ${INCLUDE_DIR}/peg_filter/plug_service.h
)

set(SRC_FILES ${SRC_DIR}/peg_sensor/plug_sensor.cpp			
	      ${SRC_DIR}/peg_filter/plug_likelihood.cpp
              ${SRC_DIR}/peg_filter/motion_model.cpp
              ${SRC_DIR}/peg_filter/pf_manager.cpp
              ${SRC_DIR}/peg_filter/plug_service.cpp
)


##  SENSOR (LIB) ##

add_library(peg_sensor ${H_FILES} ${SRC_FILES}						 )
target_link_libraries(peg_sensor ${catkin_LIBRARIES} armadillo ${LIB_STATISTICS} mlpack  )


##  PEG FILTER (EXEC) ##

add_executable(peg_filter_node ${SRC_DIR}/peg_filter/plug_filter_node.cpp      	   	)
target_link_libraries(peg_filter_node ${catkin_LIBRARIES}  peg_sensor    		)


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
