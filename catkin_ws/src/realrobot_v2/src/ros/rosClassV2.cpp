/* A class to communicate with other ros node -- real robot node 
 *
 * Author： chen chen
 * Date: 2024-4-2
 * Email: 1240563221@qq.com
 *
 */
#include "rosClassV2.h" 
#include <sys/time.h> 
namespace robot_ros_ns{
    /**  
    *   @brief      construct function of RosClass
        Parameters:
    *   @return     none
        */
    RosClass::RosClass(int argc, char** argv){
        //1.init variables
        std::string nodeName("robot");
        robot_mkey = string("ambot");
        //2.init ros node
        ros::init(argc, argv, nodeName);
        //3.create multi receive thread for topic,avoid receive data block
        spinner = new ros::AsyncSpinner(0);
        spinner->start();
        //4.check if ros master node is ok
        if(!ros::master::check()){
            ROS_ERROR("ros::master::check() did not pass!");
            ros::shutdown();
        }
        //5.instance rosNodeHandle
        node = new ros::NodeHandle();
        //6.I guest may be this variable not be used  
        // !!!!note  ros rate controlled  by ros rate item in config file, not this interface
        // ros::Duration period(1.0/100); // 100Hz update rate
        //7.init RosClass class
        if(!init()){
            ROS_ERROR("ros::master::check() did not pass!");
            ros::shutdown();
        }
        ROS_INFO("Robot RosClass Interface init successfully!");
    }

    /**  
    *   @brief      deconstruct function of RosClass
        Parameters:
    *   @return     none
        */
    RosClass::~RosClass(){
        ROS_INFO("ambot real robot node just terminated!");
        terminateValueSub.shutdown();
        modelValueSub.shutdown();
        spinner->stop();
        robot_params.clear();
        robot_devices.clear();
        delete spinner;
        delete node;
        delete rate;
        ros::shutdown();
    }
    void RosClass::rosSleep(){
        rate->sleep();
    }
    
    /**  
    *   @brief      init api of RosClass
        Parameters:
    *   @return     true:init successful; false:init failed    
        */
    bool RosClass::init()
    {
        // 1.get joint name from the parameter server
        map<string, float> tmp;
        if(!node->getParam(robot_mkey+"_params", tmp))
        {
            ROS_ERROR("robot param given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }

        // 2.get all param server params 
        map<string, float>::const_iterator iter = tmp.begin();
        for(;iter!=tmp.end();iter++)
        {
            robot_params.insert(pair<string, float>(robot_mkey+string("_params/") + iter->first, iter->second));
        }
        tmp.clear();

        // 3.checkout whether if ambot_device exist
        map<string, string> tmp1;
        if(!node->getParam(robot_mkey+"_devices", tmp1))
        {
            ROS_ERROR("robot devices given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        // 3.1 if exist, save data to variable
        map<string, string>::const_iterator iter1 = tmp1.begin();
        for(;iter1!=tmp1.end();iter1++){
            robot_devices.insert(pair<string, string>(robot_mkey+string("_devices/") + iter1->first, iter1->second));
        }
        tmp1.clear();

        // 4.1 get subscribe topics
        std::vector<std::string> subscribe_name;
        if (!node->getParam("robotSubscribeTopic", subscribe_name)){
            ROS_ERROR("No subscribe given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        // 4.2 get advertise topics
        std::vector<std::string> advertise_name;
        if (!node->getParam("robotAdvertiseTopic", advertise_name)){
            ROS_ERROR("No advertise given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }

        // 5.1 init advertise topic
        if(advertise_name.size() > 0)
            sensorValuePub = node->advertise<ambot_msgs::RobotState>(advertise_name.at(0), 1);
        if(advertise_name.size() > 1)
            terminateValuePub = node->advertise<std_msgs::Bool>(advertise_name.at(1), 1);
        
        // 5.2 init subscribe topic
        if(subscribe_name.size() > 0)
            commandValueSub = node->subscribe(subscribe_name.at(0), 1, &RosClass::commandValueCallback, this);
        // if(subscribe_name.size() > 1)
        //     commandValueSub = node->subscribe(subscribe_name.at(1), 1, &RosClass::commandValueCallback, this);
        if(subscribe_name.size() > 1)
            terminateValueSub = node->subscribe(subscribe_name.at(1), 1, &RosClass::terminateValueCallback, this);
        // 5.3 init struct values
        ambotFeatures.motorDevName = robot_devices[robot_mkey + string("_devices/motor_device")];
        ambotFeatures.sensorDevName = robot_devices[robot_mkey + string("_devices/sensor_device")];
        ambotFeatures.motorDevBaud = (unsigned int)(robot_params[robot_mkey + string("_params/motor_baud")]);
        ambotFeatures.sensorDevBaud = (unsigned int)(robot_params[robot_mkey + string("_params/sensor_baud")]);
        ambotFeatures.motorControlMode = (uint8_t)(robot_params[robot_mkey + string("_params/motor_control_mode")]);
        ambotFeatures.motorMaxAngle = (float)(robot_params[robot_mkey + string("_params/motor_max_angle")]);
        ambotFeatures.motorMinAngle = (float)(robot_params[robot_mkey + string("_params/motor_min_angle")]);
        ambotFeatures.motorVelocity = (float)(robot_params[robot_mkey + string("_params/motor_velocity")]);
        ambotFeatures.limbNum = (uint8_t)(robot_params[robot_mkey + string("_params/limb_num")]);
        ambotFeatures.legNum = (uint8_t)(robot_params[robot_mkey + string("_params/limb_num")]);
        ambotFeatures.motorNum = (uint8_t)(robot_params[robot_mkey + string("_params/motor_num")]);
        ambotFeatures.imuNum = (uint8_t)(robot_params[robot_mkey + string("_params/imu_num")]);
        ambotFeatures.forceDataNum = (uint8_t)(robot_params[robot_mkey + string("_params/force_data_num")]);
        ambotFeatures.sensorNum = (uint8_t)(robot_params[robot_mkey + string("_params/sensor_num")]);
        ambotFeatures.rosRate = (uint16_t)(robot_params[robot_mkey + string("_params/ros_rate")]);

        // 6 using ros_rate init ros rate
        rate = new ros::Rate(ambotFeatures.rosRate);
        // 7 init variable
        sensorValue.resize(ambotFeatures.sensorNum);
        modelValue.resize(ambotFeatures.motorNum);
        // motorValue.resize(ambotFeatures.motorNum);
        commandValues.motorAction.resize(ambotFeatures.motorNum);
        ambotState.motorState.resize(ambotFeatures.motorNum);

        ROS_INFO("Ambot real robot V2 interface (rostopics communicated with controller) init successful!");
        return true;
    }

    /**  
    *   @brief      get all parameters from param server and save data
        Parameters:
    *   @param      robot_params    [out]the map data include key and data
    *   @param      robot_devices   [out]the map data include key and data
    *   @return     none
        */
    void RosClass::getParameters(map<string, float>& robot_params, map<string, string>& robot_devices){

        // 1.make sure robot param exist and save data to variable
        map<string, float> tmp;
        if(!node->getParam(robot_mkey+"_params", tmp)){
            ROS_ERROR("robot param given (namespace: %s)", node->getNamespace().c_str());
        }
        map<string, float>::const_iterator iter = tmp.begin();
        for(;iter != tmp.end(); iter++){
            robot_params.insert(pair<string, float>(robot_mkey+string("_params/") + iter->first, iter->second));
        }
        tmp.clear();

        // 2.make sure robot device exist and save data to variable
        map<string, string> tmp1;
        if(!node->getParam(robot_mkey+"_devices", tmp1)){
            ROS_ERROR("robot devices given (namespace: %s)", node->getNamespace().c_str());
        }
        map<string, string>::const_iterator iter1 = tmp1.begin();
        for(;iter1!=tmp1.end();iter1++){
            robot_devices.insert(pair<string, string>(robot_mkey+string("_devices/") + iter1->first, iter1->second));
        }
        tmp1.clear();

        // 3. update all device and param to class member
        this->robot_params.clear();
        this->robot_devices.clear();
        this->robot_params.insert(robot_params.begin(), robot_params.end());
        this->robot_devices.insert(robot_devices.begin(), robot_devices.end());
    }

    /**  
    *   @brief      get single server params
        Parameters:
    *   @param      paramName   [in]the single server param name
    *   @param      getData     [out]the data need to get
    *   @return     none
        */
    bool RosClass::getSingleServerParameter(std::string paramName, int& getData)
    {
        if(!node->getParam(paramName, getData))
        {
            ROS_ERROR("get single server param failed!!! (feature: %s)", paramName.c_str());
            return false;
        }
        return true;
    }

    /**  
    *   @brief      set single server params
        Parameters:
    *   @param      robot_params    [out]the single server param name
    *   @param      setData         [out]the data need to set
    *   @return     none
        */
    template <typename T>
    void RosClass::setSingleServerParameter(string robot_params, T setData)
    {
        //set the single server param
        node->setParam(robot_params, setData);
    }


    /**  
    *   @brief      set the terminate value indicate node is alive
        Parameters:
    *   @return     none
        */
    void RosClass::setTerminateValue()
    {
        std_msgs::Bool ret;
        ret.data = true;
        terminateValuePub.publish(ret);
    }

    // pub sensor values
    /**  
    *   @brief      take the feedback data of motor and sensor data send to topic
        Parameters:
    *   @param      data    [in]sensor data
    *   @return     none
        */
    void RosClass::stateDataPublish(void)
    {   
        ambot_msgs::RobotState values;
        values = ambotState;
        sensorValuePub.publish(values);
    }
    
    /**  
    *   @brief      output the motor data to argv
        Parameters:
    *   @param      data    [in]receive the motor data
    *   @return     none
        */
    void RosClass::motorCommandOutput(ambot_msgs::RobotAction& data){
        assert(commandValues.motorAction.size() >= ambotFeatures.motorNum);
        data = commandValues;
        for (int i = 0; i < data.motorAction.size(); i++)
        {
            if (i%3 ==0 || i== 2 || i == 8)
            {
                data.motorAction[i].q = -data.motorAction[i].q;
                data.motorAction[i].dq = -data.motorAction[i].dq;
                data.motorAction[i].tor = -data.motorAction[i].tor;
            }
        }
    
        //set command offset
        // swing state, 0, 90, and 90du
//        data.motorAction[0].q += 0;
//        data.motorAction[1].q += MY_PI / 2;
//        data.motorAction[2].q -= MY_PI / 2;
//        data.motorAction[3].q += 0;
//        data.motorAction[4].q -= MY_PI / 2;
//        data.motorAction[5].q += MY_PI / 2;
//        data.motorAction[6].q += 0;
//        data.motorAction[7].q += MY_PI / 2;
//        data.motorAction[8].q -= MY_PI / 2;
//        data.motorAction[9].q += 0;
//        data.motorAction[10].q -= MY_PI / 2;
//        data.motorAction[11].q += MY_PI / 2;
        // ground state, 0, 35, and 60du
        data.motorAction[0].q += 0;
        data.motorAction[1].q -= 35.0/180.0*MY_PI;
        data.motorAction[2].q += 60.0/180.0*MY_PI;
        data.motorAction[3].q += 0;
        data.motorAction[4].q += 35.0/180.0*MY_PI;
        data.motorAction[5].q -= 60.0/180.0*MY_PI;
        data.motorAction[6].q += 0;
        data.motorAction[7].q -= 35.0/180.0*MY_PI;
        data.motorAction[8].q += 60.0/180.0*MY_PI;
        data.motorAction[9].q += 0;
        data.motorAction[10].q+= 35.0/180.0*MY_PI;
        data.motorAction[11].q-= 60.0/180.0*MY_PI;

        for (int i = 0; i < data.motorAction.size(); i++)
        {
            if (i%3 > 0){
                data.motorAction[i].q = -data.motorAction[i].q;
                data.motorAction[i].dq = -data.motorAction[i].dq;
                data.motorAction[i].tor = -data.motorAction[i].tor;
            }
        }

    }

    /**  
    *   @brief      output the motor data to argv
        Parameters:
    *   @param      data    [in]receive the motor data
    *   @return     none
        */
    void RosClass::motorCommandOutput(std::vector<float>& data){
        assert(data.size() == ambotFeatures.motorNum);
        data = modelValue;
    }

    /**  
    *   @brief      the callback of receive motor data function
        Parameters:
    *   @param      array    [in]the data receive from the locomotion controller for each joint 
    *   @return     none
        */
    void RosClass::modelValueCallback(const std_msgs::Float32MultiArray array){
        assert(array.data.size() == ambotFeatures.motorNum);
        for(uint8_t idx = 0;idx < ambotFeatures.motorNum; idx++)
        {
            modelValue[idx] = array.data[idx];
        }
    }

    /**  
    *   @brief      the callback of receive motor data function
        Parameters:
    *   @param      array    [in]the data receive from the locomotion controller for each joint 
    *   @return     none
        */
    void RosClass::commandValueCallback(const ambot_msgs::RobotAction array){
        assert(array.motorAction.size() <= ambotFeatures.motorNum);
        for(uint8_t idx = 0;idx < ambotFeatures.motorNum; idx++)
        {
            commandValues = array;
        }
    }

    /**  
    *   @brief      the callback of terminate function
        Parameters:
    *   @param      termNode    [in]the data could judge whether need to close current ros node
    *   @return     none
        */
    void RosClass::terminateValueCallback(const std_msgs::Bool& termNode)
    {
        terminate = (bool)termNode.data;
    }

    /**  
    *   @brief      update motor value
        Parameters:
    *   @return     none
        */
    void RosClass::updateMotorValue(){
        for(uint8_t idx = 0;idx < ambotFeatures.motorNum; idx++)
            motorValue[idx] = modelValue[idx];
    }

    void RosClass::imuDataUpdate(const sensor_msgs::Imu& data)
    {
        ambotState.imu.acceleration.x = data.linear_acceleration.x;
        ambotState.imu.acceleration.y = data.linear_acceleration.y;
        ambotState.imu.acceleration.z = data.linear_acceleration.z;
        ambotState.imu.gyroscope.x = data.angular_velocity.x;
        ambotState.imu.gyroscope.y = data.angular_velocity.y;
        ambotState.imu.gyroscope.z = data.angular_velocity.z;
        ambotState.imu.quaternion.w = data.orientation.w;
        ambotState.imu.quaternion.x = data.orientation.x;
        ambotState.imu.quaternion.y = data.orientation.y;
        ambotState.imu.quaternion.z = data.orientation.z;
    }

    void RosClass::motorDataUpdate(uint8_t motorNum,const std::vector<ambot_msgs::JointState>& data)
    {
        ambotState.motor_num = data.size();
        for (int i = 0; i < data.size(); i++)
        {
            ambotState.motorState[i] = data[i];
        }
        
        // ambotState.motorState = data;
        // printf("pos:%f, vel:%f, cur:%f\n", data[1].pos, data[1].vel, data[1].cur);
    }

    void RosClass::ambotDataUpdate(const ambot_msgs::RobotState& data)
    {
        static const float bias[12] = { 0, 35.0/180.0*M_PI, -60.0/180.0*M_PI,
                                        0, -35.0/180.0*M_PI, 60.0/180.0*M_PI,
                                        0, 35.0/180.0*M_PI, -60.0/180.0*M_PI,
                                        0, -35.0/180.0*M_PI, 60.0/180.0*M_PI,};
        assert(data.motorState.size() <= ambotFeatures.motorNum);
        ambotState = data;
	for (int i = 0; i < data.motorState.size(); i++)
	{
            if (i%3 > 0)
		{
		ambotState.motorState[i].pos = -ambotState.motorState[i].pos;
		ambotState.motorState[i].vel = -ambotState.motorState[i].vel;
		ambotState.motorState[i].cur = -ambotState.motorState[i].cur;
		ambotState.motorState.at(i).pos += bias[i];
            }
        }

	for (int i = 0; i < data.motorState.size(); i++)
        {
            if (i%3 ==0 || i== 2 || i == 8)
            {
		ambotState.motorState[i].pos = -ambotState.motorState[i].pos;
		ambotState.motorState[i].vel = -ambotState.motorState[i].vel;
		ambotState.motorState[i].cur = -ambotState.motorState[i].cur;
            }
        }
    }

    /**  
    *   @brief      check current program update frequency and judge whether output warning message
        Parameters:
    *   @return     none 
        */
    void RosClass::topicFrequencyWarning(void)
    {
        //1.get current time and init variables
        static long int deltaTime = 0;
        static bool firstFlag = true;
        static struct timeval time_old ,time_new;
        gettimeofday(&time_new, NULL);
        //2.get the internal time between two update
        deltaTime = (time_new.tv_sec*1000000 + time_new.tv_usec)-(time_old.tv_sec*1000000 + time_old.tv_usec);
        //3.check whether the deltaTime greater than the setting ros_rate internal time
        //compare data not frequency,but the period time,according to the equation;f=1/T to calculate
        //if calculate result internal time less than internal of ros_rate/3 frequency,than output warning message
        if((deltaTime > 3*1000000 / ambotFeatures.rosRate) && !firstFlag)
        {
            printf("delta:%ld, internal:%.3f, ros_rate:%d\n", deltaTime, (double)(3*1000000 / ambotFeatures.rosRate), ambotFeatures.rosRate);
            ROS_WARN("the frequency of Ros topic publish is too slow, frequency:%.3f", (double)(1000000/deltaTime));
        }            
        gettimeofday(&time_old, NULL);
        if(firstFlag)
        {
            firstFlag = false;
        }
    }

    /**  
    *   @brief      get ros handle
        Parameters:
    *   @return     return the pointer of ros handle of RosClass   
        */
    ros::NodeHandle* RosClass::getHandle(){
        return node;
    }
}//namespace
