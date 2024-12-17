#include <iostream>
#include <ros/ros.h>
#include "robot.h"
using namespace robot_ns;
using namespace robot_ros_ns;

int main(int argc, char** argv){
    Robot robot(argc, argv);
    while(robot.run()){
        }
    return 0;

}
