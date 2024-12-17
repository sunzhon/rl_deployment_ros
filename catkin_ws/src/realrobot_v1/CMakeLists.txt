cmake_minimum_required(VERSION 2.8.3)
project(realrobot_v1)
SET (CMAKE_CXX_STANDARD 14)
SET(REAL_ROBOTS "${CMAKE_CURRENT_SOURCE_DIR}/src")
SET(IMU_FOLDER "${REAL_ROBOTS}/imu/")
SET(UTILS_FOLDER "${REAL_ROBOTS}/utils")
SET(BOTA_FOLDER "${REAL_ROBOTS}/bota")
SET(IMU_SRCS "${IMU_FOLDER}/src")
SET(IMU_ARCH_SRCS "${IMU_FOLDER}/src/arch/linux")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Debug")
#if(NOT CMAKE_CXX_STANDARD)
#    set(CMAKE_CXX_STANDARD 14)
#endif()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# add_definitions("-std=c++17")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  dynamixel_workbench_controllers
  sensor_msgs
  ambot_msgs
  diagnostic_updater
)

## System dependencies are found with CMake's conventions
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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imu_vn_100
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
  DEPENDS 
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  ambot_msgs
  serial
)

###########
## Build ##
###########

add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_DEBUG)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   include
#   include/ambot/
#   ${catkin_INCLUDE_DIRS}
# )
include_directories(
  ${CMAKE_SOURCE_DIR}/include 
  ${REAL_ROBOTS}/include
  ${CMAKE_SOURCE_DIR}/include/ambot/
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/dynamixel_sdk
  ${IMU_FOLDER}/include
  ${UTILS_FOLDER}
  ${BOTA_FOLDER}
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ambot.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ambot_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

## Specify libraries to link a library or executable target against
aux_source_directory(${CMAKE_SOURCE_DIR}/src SRCS)
aux_source_directory(${BOTA_FOLDER} BOTA_SRCS)
aux_source_directory(${REAL_ROBOTS}/src REAL_ROBOTS_SRCS)
aux_source_directory(${UTILS_FOLDER} UTILS)
# aux_source_directory(${IMU_FOLDER}/src IMU_SRCS)
# aux_source_directory(${IMU_FOLDER}/src/arch/linux IMU_ARCH_SRCS)

add_library(imu_vn_100
            ${IMU_ARCH_SRCS}/vncp_services.c
            ${IMU_SRCS}/vndevice.c
            ${IMU_SRCS}/vn100.c
            ${IMU_SRCS}/imu_vn_100.cpp)   
target_link_libraries(imu_vn_100
                      ${catkin_LIBRARIES})  

add_dependencies(imu_vn_100
                ${${PROJECT_NAME}_EXPORTED_TARGETS}
                ${catkin_EXPORTED_TARGETS})

message("SRCS=${SRCS}")

add_executable(${PROJECT_NAME}_node  
              ${SRCS} 
              ${REAL_ROBOTS_SRCS}
              ${BOTA_SRCS}
	      ${UTILS}
	      )

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} imu_vn_100)
add_dependencies(${PROJECT_NAME}_node 
                  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                  ${catkin_EXPORTED_TARGETS})



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
 install(TARGETS  ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/
 )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ambot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
