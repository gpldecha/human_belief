cmake_minimum_required(VERSION 2.8.3)
project(human_belief)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED world_wrapper wrap_object objects
optitrack_rviz tf std_msgs particle_filter table_wrapper)

find_package(Boost)
find_package(Armadillo)

if(ARMADILLO_FOUND)
    message(STATUS "${Green} libarmadillo found ${ColourReset} "            )
    message(STATUS "    ${ARMADILLO_INCLUDE_DIRS}"                          )
    message(STATUS "    ${ARMADILLO_LIBRARIES}"                             )
elseif(ARMADILLO_FOUND)
    message(FATAL_ERROR "${Red} libarmadillo NOT FOUND! ${ColourReset}"     )
endif(ARMADILLO_FOUND)

#-fopenmp

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-unused-variable -Wno-deprecated-declarations -fopenmp")        ## Optimize
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

add_service_files(
   FILES
   String_cmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  LIBRARIES peg_in_hole
  CATKIN_DEPENDS message_runtime world_wrapper objects
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})

set(H_FILES  	${INCLUDE_DIR}/human_belief/human_belief.h
                ${INCLUDE_DIR}/human_belief/human_likelihood.h
                ${INCLUDE_DIR}/human_belief/human_measurement.h


)

set(SRC_FILES 	${SRC_DIR}/human_belief/human_belief.cpp
                ${SRC_DIR}/human_belief/human_likelihood.cpp
                ${SRC_DIR}/human_belief/human_measurement.cpp

)

# Human belief REPLAY (LIB)

add_library(belief_replay SHARED ${H_FILES} ${SRC_FILES}                          		)
target_link_libraries(belief_replay ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} boost_system	)

add_library(trajectory_replay SHARED ${SRC_DIR}/replay_trajectory.cpp                           )
target_link_libraries(trajectory_replay ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} boost_system	)


add_executable(replay_belief_node ${SRC_DIR}/replay_belief.cpp                           )
target_link_libraries(replay_belief_node ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} boost_system belief_replay)


#add_executable(human_console ${INCLUDE_DIR}/human_belief/human_console.h ${SRC_DIR}/human_belief/human_console.cpp)
#target_link_libraries(human_console ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} boost_system belief_replay)

## Declare a C++ library
# add_library(human_belief
#   src/${PROJECT_NAME}/human_belief.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(human_belief ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(human_belief_node src/human_belief_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(human_belief_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(human_belief_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS human_belief human_belief_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_human_belief.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
