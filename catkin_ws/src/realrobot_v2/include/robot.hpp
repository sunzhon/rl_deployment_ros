/*
 * robot.hpp
 * This is 2g ambot robot run program
 * Created on: march 1, 2024
 *      Author: chen chen
 */
#ifndef __ROBOT_HPP__
#define __ROBOT_HPP__

#include "ambotDriver.hpp"
#include "rosClassV2.h"

class Robot
{
private:
    /* data */
    robot_ros_ns::RosClass *ros;
    AmbotDriver* ambot;
    ambot_msgs::RobotAction ambotCommand;
    bool imuConnectFlag = false;
public:
    Robot(int argc, char** argv);
    ~Robot();
    bool init(void);
    void runEnd(void);
    void run(void);
    void setThreadRunFlag(void);
};
#endif
