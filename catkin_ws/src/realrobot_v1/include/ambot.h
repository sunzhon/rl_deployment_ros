#ifndef __AMBOT_H__
#define __AMBOT_H__
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ros/ros.h"
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "ambot_msgs/RobotState.h"
#include "ambot_msgs/RobotAction.h"
#include <sstream>
#include <algorithm>
#include "typeHeader.h"
#include "dynamixelClass.h"
#include "imu_vn_100.h"
#include "bota_force_sensor.h"
#include <vector>
#include <utility>


using namespace std;

#define MAX_NUMBER_MOTOR 100

namespace robot_ns{

    class Ambot{
        public:
            Ambot();
            ~Ambot();    
            bool init(map<string,float>& robot_params, map<string, string>& robot_devices);

            void setMotorValue(const vector<command>& value);
            void setMotorValue(const ambot_msgs::RobotAction& value);

            void getSensoryValue(vector<sensor>& value);
            void getSensoryValue(ambot_msgs::RobotState& value);
            void getParameters();
            void storeData(std::vector<std::pair<string, float>>& data);
        private:
            void initMsg(); 
            bool set_motors(); 
            void readJoints();
            void readImu();
            void readFootForce();
            void writeServoValue();
            void readSensorValue();
            void localController();
            void value2Position(const std::vector<float>& value, std::vector<int32_t>& cmd);

        private:
            #ifdef DYNAMIXEL_MODE_WORKBENCH
            DynamixelWorkbench* dxl_wb;
            #else
            DynamixelClass * dxl_wb;
            #endif
            imu_vn_100::ImuVn100* imuSensor;
            bota_ns::BotaSensor* forceSensor;

            int return_delay_time;
            int profile_acceleration;
            int profile_velocity;
            bool imuConnectFlag = false;
            bool forceConnectFlag = false;
            uint8_t dxl_id[MAX_NUMBER_MOTOR];
            uint8_t dxl_cnt;
            long int t;
            //deivce num
            //feedback sensory with physical unit
            std::vector<sensor> physical_present_position;
            std::vector<sensor> physical_present_velocity;
            std::vector<sensor> physical_present_current;
            std::vector<sensor> physical_present_voltage;

            std::vector<sensor> pose;
            std::vector<sensor> grf;
            std::vector<sensor> sensorValue;
            std::vector<sensor> motorValue;

            //dxl device
            std::vector<int32_t> goal_position;
            std::vector<int32_t> goal_velocity;
            std::vector<int32_t> goal_current;

            std::vector<int32_t> present_position;
            std::vector<int32_t> present_velocity;
            std::vector<int32_t> present_current; 
            std::vector<int32_t> present_voltage;

            //local controller
            std::vector<int32_t> position_error;
            std::vector<int32_t> previous_goal_position;
            std::vector<float> float_position_error;
            std::vector<float> float_velocity_error;
            // The gains of the torque lilimit control 
            float p_gain;
            float d_gain;
            // sensory feedback results through seril port and low-level-control board
            std_msgs::String eboard_sensory_results;
        public:
            int leg_num;
            int motor_num;
            int sensor_num;
            int action_num;
            int pose_num;
            int force_data_num;
    };
int angle2Position(double angle, bool isRadians);

}

#endif

