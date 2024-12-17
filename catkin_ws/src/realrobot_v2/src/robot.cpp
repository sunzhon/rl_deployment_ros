/*
 * robot.cpp
 * This is 2g ambot robot run program
 * Created on: march 1, 2024
 *      Author: chen chen
 */
#include "robot.hpp"
/**  
*   @brief      robot default construct function
    Parameters:
*   @param      
*   @return     none
    */
Robot::Robot(int argc, char** argv)
{
    ros = new robot_ros_ns::RosClass(argc, argv);
}

/**  
*   @brief      robot deconstruct function
    Parameters:
*   @param      
*   @return     none
    */
Robot::~Robot()
{
    delete ambot;
}

/**  
*   @brief      robot init function
    Parameters:
*   @param      
*   @return     none
    */
bool Robot::init(void)
{
    //1 init driver
    ambot = new AmbotDriver(ros->ambotFeatures);
    if(ambot->initial())
        return true;
    else
        return false;
}

/**  
*   @brief      robot run end function
    Parameters:
*   @param      
*   @return     none
    */
void Robot::runEnd(void)
{
    ambot->disableAllMotor();
}

/**  
*   @brief      robot run step
    Parameters:
*   @param      
*   @return     none
    */
void Robot::run(void)
{
    ros->ambotDataUpdate(ambot->ambotState);
    ros->stateDataPublish();
    ros->motorCommandOutput(ambotCommand);
    if(!ambot->threadStop)
    {
        ambot->setMotorLocomotionCommand(ambotCommand);
    }else
    {
        ros->setTerminateValue();
    }
    ros->rosSleep();
}

/**  
*   @brief      set thread stop flag
    Parameters:
*   @param      
*   @return     none
    */
void Robot::setThreadRunFlag(void)
{
    ambot->threadStop = true;
}