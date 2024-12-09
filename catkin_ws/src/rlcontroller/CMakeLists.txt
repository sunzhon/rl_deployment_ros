cmake_minimum_required(VERSION 2.8.3)
project(rlcontroller)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors ")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
SET (CMAKE_CXX_STANDARD 11)
SET (GOROBOTS "../../../../../..")
#SET (LPZROBOTS "../../../../../../../lpzrobots")

SET (CONTROLLERS "${GOROBOTS}/controllers")
SET (UTILS "${GOROBOTS}/utils")
#SET (SELFORG "${LPZROBOTS}/selforg" )
#SET (ODE_ROBOTS "${LPZROBOTS}/ode_robots")

#SET(DEFINES "-DQT_DEPRECATED_WARNINGS -DQT_NO_DEBUG -DQT_CHARTS_LIB -DQT_WIDGETS_LIB -DQT_GUI_LIB -DQT_CORE_LIB")
#SET(CXXFLAGS "-m64 -pipe -D_REENTRANT -fPIC ${DEFINES}")

#SET(CMAKE_CXX_FLAGS_DEBUG"$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE"$ENV{CXXFLAGS} -O3 -Wall")
SET(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  control_msgs
  geometry_msgs
  rospy
  message_generation
)

# Find the Shark libraries and includes
# set Shark_DIR to the proper location of Shark
#find_package(Shark REQUIRED)
#include(${SHARK_USE_FILE})
#if(CMAKE_COMPILER_IS_GNUCXX)
#message(STATUS "optional: ")
#endif()

# System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
#add_message_files(
#    FILES
#    AmbotState.msg
#    ImuState.msg
#    JointState.msg
#    imu_base_msg.msg
#)
#generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs  message_runtime
    DEPENDS
)


#
#
#
##
##
#
### Generate services in the 'srv' folder
## add_service_files(
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
#   std_msgs
# )

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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rlcontroller
#  CATKIN_DEPENDS roscpp std_msgs sensor_msgs control_msgs
#  DEPENDS system_lib
#)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ${CXXFLAGS} ${QT5_INPATH}")
set(CMAKE_CXX_FLAGS "-fext-numeric-literals ${CMAKE_CXX_FLAGS}")
set(CNAKE_VERBOSE_MAKEFILE ON)
## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  "${catkin_INCLUDE_DIRS}"
)
#link_directories(${SELFORG} 
    #${ODE_ROBOTS} "$ENV{HOME}/lib" 
    #"${LPZROBOTS}/configurator"
    #)

#FIND_LIBRARY(SELFORG_LIB selforg ${SELFORG} NO_DEFAULT_PATH)
#FIND_LIBRARY(ODE_ROBOTS_LIB ode_robots ${ODE_ROBOTS} NO_DEFAULT_PATH)

## Declare a C++ library
# add_library(${CONTROLLER_LIBS}
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
AUX_SOURCE_DIRECTORY( ./src TEST_SRCS)


#add_definitions("-Wall -g -std=c++11")
#add_executable(${PROJECT_NAME}_node src/stbot.cpp
#   ${UTILS_SRCS}
#   ${CONTROLLER_SRCS}
#   ${CONTROLLER_LIB_SRCS}
#   ${UTILS_ANN_FRAMEWORK_SRCS} 
#   ${UTILS_ANN_LIBRARY_SRCS}
#   # ${UTILS_ESN_FRAMEWORK_SRCS} 
#   ${UTILS_STUDYTOOL_SRCS}
#   ${UTILS_LEARNING_TOOL_SRCS}
#)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )


## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   #${SELFORG_LIB}
#   #   ${ODE_ROBOTS_LIB}
#   gsl gslcblas
#   #${SHARK_LIBRARIES}
#   #ncurses ode_robots m osgShadow osgText osgUtil osgViewer osgGA osgDB OpenThreads osg GL GLU glut ode_dbl  readline pthread selforg   gsl gslcblas configurator  QtCore QtGui QtXml libselforg.a 
# )

#ncurses ode_robots m osgShadow osgText osgUtil osgViewer osgGA osgDB OpenThreads osg GL GLU glut ode_dbl  readline pthread selforg   gsl gslcblas configurator  QtCore QtGui QtXml libselforg.a 

##	Qt5Core Qt5Gui Qt5Charts Qt5Widgets 

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
#install(TARGETS ${PROJECT_NAME}_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
#SET (CATKIN_PACKAGE_CONFIG_DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/../../config/")
#SET (CATKIN_PACKAGE_LAUNCH_DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/../../launch/")
# install(FILES
#    launch/stbot.launch
#   DESTINATION ${CATKIN_PACKAGE_LAUNCH_DESTINATION}
# )
#install(FILES                   
#    config/stbot.yaml
#    config/stbot_env.zsh
#
#   DESTINATION ${CATKIN_PACKAGE_CONFIG_DESTINATION}
#)
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_stbot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
