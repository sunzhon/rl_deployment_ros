cmake_minimum_required(VERSION 3.0.2)
project(realrobot_v2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(show_read_feedback_period disable)

set(MOTOR_SRC "./src/motor")
set(ROS_SRC "./src/ros")
set(MOTOR_INC "./include/motor")
set(ROS_INC "./include/ros")

if( "${show_read_feedback_period}" STREQUAL "enable" )
#add a macro to project
    message("attention, controller will use tensorboard_logger to save data!!!!!")
    add_definitions(-DSHOW_READ_FEEDBACK_PERIOD)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   serial
# )
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    geometry_msgs 
    serial
    ambot_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_serial
    # CATKIN_DEPENDS roscpp rospy std_msgs 
#  DEPENDS system_lib serial
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${MOTOR_INC}
    ${ROS_INC}
    "/usr/local/include/eigen3/"
    "/usr/local/include/"
)

# set(SRC ${CMAKE_SOURCE_DIR}/src)

aux_source_directory(${CMAKE_SOURCE_DIR}/src SRC)
aux_source_directory(${MOTOR_SRC} MOTOR_FILES)
aux_source_directory(${ROS_SRC} ROS_FILES)

message("src ${SRC}")
message("motor ${MOTOR_FILES}")
message("ros ${ROS_FILES}")

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/test_serial.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
                ${SRC} 
                ${MOTOR_FILES}
                ${ROS_FILES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

