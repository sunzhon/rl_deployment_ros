#ifndef __ROS_CLASS_AMBOT_H__
#define __ROS_CLASS_AMBOT_H__

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <pluginlib/class_list_macros.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Bool.h"
//#include <urdf/model.h>
#include <std_msgs/Float32MultiArray.h>
#include  "ambot_msgs/RobotState.h"
#include  "ambot_msgs/RobotAction.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <control_msgs/JointJog.h>
#include <ros/spinner.h>
#include "typeHeader.h"
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

            ros::Subscriber cpgValueSub;
            ros::Subscriber modelValueSub;
            ros::Subscriber reflexValueSub;
            ros::Subscriber terminateValueSub;
            ros::Subscriber actionValueSub;

            void cpgValueCallback(const std_msgs::Float32MultiArray array);
            void reflexValueCallback(const std_msgs::Float32MultiArray array);
            void modelValueCallback(const std_msgs::Float32MultiArray array);
            void terminateValueCallback(const std_msgs::Bool& termNode);

            void actionValueCallback(const ambot_msgs::RobotAction& msg);
	    ambot_msgs::RobotState motorState;
	    ambot_msgs::RobotAction motorAction;


            void updateMotorValue();
            ros::AsyncSpinner* spinner;
            ros::NodeHandle* node;
            ros::Rate * rate;
            std::vector<command> motorValue;
            std::vector<command> reflexValue;
            std::vector<command> cpgValue;
            std::vector<command> modelValue;
            std::vector<command> sensorValue;
            unsigned int ros_rate;
        public:
            void topicFrequencyWarning(void);

            void pubSensorValue(const ambot_msgs::RobotState& data);
            void fetchMotorValue(ambot_msgs::RobotAction& data);

            void readSensorValue(const std::vector<sensor>& data);
            void writeMotorValue(std::vector<command>& data);
            void setTerminateValue();
            void getParameters(map<string, float>& robot_params, map<string, string>& robot_devices);
            void setSingleServerParameter(string robot_params, int setData);
            bool getSingleServerParameter(std::string paramName, int& getData);
            ros::NodeHandle* getHandle();
            int motor_num;
            int action_num;
            int motor_data_num;
            int sensor_num;
            bool terminate=false;
            std::map<string, float> robot_params;
            std::map<string, string> robot_devices;
            string robot_mkey;
            bool recive_actionvalue_flag;

    };
    
}//namespace

#endif
