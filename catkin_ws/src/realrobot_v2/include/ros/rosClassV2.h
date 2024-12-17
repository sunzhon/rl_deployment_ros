#ifndef __ROS_CLASS_V2_H__
#define __ROS_CLASS_V2_H__

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <ambot_msgs/RobotState.h>
#include <ambot_msgs/RobotAction.h>
#include <sensor_msgs/Imu.h>
#include <ambot_msgs/JointState.h>
#include "struct.h"
//if the include has red wavy line, you should enter ctrl+shift+P 
//and enter C/Cpp: Edit configurations chose to edit compiler tools 
//and add include path ,then you will find the red wavy line will gone.
namespace robot_ros_ns{
    using namespace std;
    class RosClass{
        public:
            RosClass(int argc,char** argv);
            ~RosClass();
            bool init();
            void rosSleep();

        private:
            ros::Publisher sensorValuePub;
            ros::Publisher terminateValuePub;

            ros::Subscriber commandValueSub;
            ros::Subscriber modelValueSub;
            ros::Subscriber terminateValueSub;

            ros::AsyncSpinner* spinner;
            ros::NodeHandle* node;
            ros::Rate * rate;
            std::vector<float> motorValue;
            std::vector<float> modelValue;
            std::vector<float> sensorValue;
            ambot_msgs::RobotAction commandValues;

            void commandValueCallback(const ambot_msgs::RobotAction array);
            void modelValueCallback(const std_msgs::Float32MultiArray array);
            void terminateValueCallback(const std_msgs::Bool& termNode);
            void updateMotorValue();

        public:
            bool terminate = false;
            std::map<string, float> robot_params;
            std::map<string, string> robot_devices;
            string robot_mkey;
            AmbotDeviceTPDF ambotFeatures;
            ambot_msgs::RobotState ambotState;

            void imuDataUpdate(const sensor_msgs::Imu& data);
            void motorDataUpdate(uint8_t motorNum, const std::vector<ambot_msgs::JointState>& data);
            void ambotDataUpdate(const ambot_msgs::RobotState& data);
            void stateDataPublish(void);
            void motorCommandOutput(ambot_msgs::RobotAction& data);
            void motorCommandOutput(std::vector<float>& data);
            void setTerminateValue();
            void getParameters(map<string, float>& robot_params, map<string, string>& robot_devices);
            template <typename T> void setSingleServerParameter(string robot_params, T setData);
            bool getSingleServerParameter(std::string paramName, int& getData);
            ros::NodeHandle* getHandle();
            void topicFrequencyWarning(void);
    };
    
}//namespace

#endif
