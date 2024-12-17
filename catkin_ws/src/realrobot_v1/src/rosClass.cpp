/*
Author:Chen chen
Date:2023-12-5
Description: Ambot Ros Interface
Modified by Tao Sun on 2024-4-15
*/
#include "rosClass.h"
#include <sys/time.h>
#define PI 3.1415926
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
		//6.init RosClass class
		if(!init()){
			ROS_ERROR("ros::master::check() did not pass!");
			ros::shutdown();
		}
		recive_actionvalue_flag=false;
		ROS_INFO("Robot RosClass Interface init successfully!");
	}

	/**  
	 *   @brief      deconstruct function of RosClass
Parameters:
	 *   @return     none
	 */
	RosClass::~RosClass(){
		ROS_INFO("ambot real robot node just terminated!");
		cpgValueSub.shutdown();
		reflexValueSub.shutdown();
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
		// get joint name from the parameter server
		map<string, float> tmp;
		if(!node->getParam(robot_mkey+"_params", tmp)){
			ROS_ERROR("robot param given (namespace: %s)", node->getNamespace().c_str());
			return false;
		}
		map<string, float>::const_iterator iter = tmp.begin();
		for(;iter!=tmp.end();iter++){
			robot_params.insert(pair<string, float>(robot_mkey+string("_params/") + iter->first, iter->second));
		}
		tmp.clear();

		map<string, string> tmp1;
		if(!node->getParam(robot_mkey+"_devices", tmp1)){
			ROS_ERROR("robot devices given (namespace: %s)", node->getNamespace().c_str());
			return false;
		}
		map<string, string>::const_iterator iter1 = tmp1.begin();
		for(;iter1!=tmp1.end();iter1++){
			robot_devices.insert(pair<string, string>(robot_mkey+string("_devices/") + iter1->first, iter1->second));
		}
		tmp1.clear();

		// subscribe topics
		std::vector<std::string> subscribe_name;
		if (!node->getParam("robotSubscribeTopic", subscribe_name)){
			ROS_ERROR("No subscribe given (namespace: %s)", node->getNamespace().c_str());
			return false;
		}
		// advertise topics
		std::vector<std::string> advertise_name;
		if (!node->getParam("robotAdvertiseTopic", advertise_name)){
			ROS_ERROR("No advertise given (namespace: %s)", node->getNamespace().c_str());
			return false;
		}

		//a) pub 
		//i) pub sensor
		if(advertise_name.size()>0)
			sensorValuePub=node->advertise<ambot_msgs::RobotState>(advertise_name.at(0), 1);
		//ii) pub terminate
		if(advertise_name.size()>1)
			terminateValuePub=node->advertise<std_msgs::Bool>(advertise_name.at(1), 1);

		//b) subscirber
		//i) Start cpgCommand subscriber
		if(subscribe_name.size()>0)
			cpgValueSub= node->subscribe<std_msgs::Float32MultiArray>(subscribe_name.at(0), 1, &RosClass::cpgValueCallback, this);

		//ii) Start cpgCommand subscriber
		if(subscribe_name.size()>1)
			reflexValueSub= node->subscribe<std_msgs::Float32MultiArray>(subscribe_name.at(1), 1, &RosClass::reflexValueCallback, this);

		//iii) subsriber model_based_control commands
		if(subscribe_name.size()>2)
			modelValueSub= node->subscribe(subscribe_name.at(2), 1, &RosClass::modelValueCallback, this);

		//iv) subsriber terminate
		if(subscribe_name.size()>3)
			terminateValueSub= node->subscribe(subscribe_name.at(3), 1, &RosClass::terminateValueCallback, this);

		//v) subsriber terminate
		if(subscribe_name.size()>4)
			actionValueSub= node->subscribe(subscribe_name.at(4), 1, &RosClass::actionValueCallback, this);


		// instantilize rate
		ros_rate = (unsigned int)(robot_params[robot_mkey+string("_params/ros_rate")]);
		rate = new ros::Rate(ros_rate);
		//init variable
		motor_num = (unsigned int)(robot_params[robot_mkey+string("_params/motor_num")]);
		action_num = (unsigned int)(robot_params[robot_mkey+string("_params/action_num")]);
		sensor_num = (unsigned int)(robot_params[robot_mkey+string("_params/sensor_num")]);
		cpgValue.resize(motor_num);
		reflexValue.resize(motor_num);
		sensorValue.resize(sensor_num);
		modelValue.resize(motor_num);
		motorValue.resize(motor_num);

		motorAction.motorAction.resize(action_num);

		ROS_INFO("Ambot controller interface (rostopics communicated with controller) init successful!");
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

		// get joint name from the parameter server
		map<string, float> tmp;
		if(!node->getParam(robot_mkey+"_params", tmp)){
			ROS_ERROR("robot param given (namespace: %s)", node->getNamespace().c_str());
		}
		map<string, float>::const_iterator iter = tmp.begin();
		for(;iter!=tmp.end();iter++){
			robot_params.insert(pair<string, float>(robot_mkey+string("_params/") + iter->first, iter->second));
		}
		tmp.clear();

		map<string, string> tmp1;
		if(!node->getParam(robot_mkey+"_devices", tmp1)){
			ROS_ERROR("robot devices given (namespace: %s)", node->getNamespace().c_str());
		}
		map<string, string>::const_iterator iter1 = tmp1.begin();
		for(;iter1!=tmp1.end();iter1++){
			robot_devices.insert(pair<string, string>(robot_mkey+string("_devices/") + iter1->first, iter1->second));
		}
		tmp1.clear();

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
	void RosClass::setSingleServerParameter(string robot_params, int setData)
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
		//   ret.data=false;
		ret.data=true;
		terminateValuePub.publish(ret);
	}

	// pub sensor values
	/**  
	 *   @brief      take the feedback data of motor and sensor data send to topic
Parameters:
	 *   @param      data    [in]sensor data
	 *   @return     none
	 */
	void RosClass::readSensorValue(const std::vector<sensor>& data)
	{   
		assert(data.size()==sensor_num);
		std_msgs::Float32MultiArray values;
		values.data.resize(sensor_num);
		for(uint8_t idx=0;idx<sensor_num;idx++)
		{
			values.data[idx]=data[idx];
		}
		sensorValuePub.publish(values);
	}





	// pub sensor values
	/**  
	 *   @brief      take the feedback data of motor and sensor data send to topic
Parameters:
	 *   @param      data    [in]sensor data
	 *   @return     none
	 */
	void RosClass::pubSensorValue(const ambot_msgs::RobotState& data)
	{   
		sensorValuePub.publish(data);
	}

	// fetch value from subscribler callback function
	void RosClass::fetchMotorValue(ambot_msgs::RobotAction& data){
		assert(data.motor_num==motorAction.motor_num);
		for(uint8_t idx=0; idx<action_num; idx++){
		data.motorAction[idx].q = motorAction.motorAction[idx].q;
	}
	}
	// action subscriber callback
	void RosClass::actionValueCallback(const ambot_msgs::RobotAction& msg){
		assert(msg.motor_num==action_num);
		recive_actionvalue_flag = true;
		for(uint8_t idx=0;idx<action_num;idx++)
		{
			motorAction.motorAction[idx].q = msg.motorAction[idx].q;
		}
	}




	/**  
	 *   @brief      output the motor data to argv
Parameters:
	 *   @param      data    [in]receive the motor data
	 *   @return     none
	 */
	void RosClass::writeMotorValue(std::vector<command>& data){
		assert(data.size()==motor_num);
		updateMotorValue();
		for(uint8_t idx=0;idx<motor_num;idx++)
			data[idx] = motorValue[idx];
	}

	void RosClass::cpgValueCallback(const std_msgs::Float32MultiArray array){
		assert(array.data.size()==motor_num);
		for(uint8_t idx=0;idx<motor_num;idx++)
			cpgValue[idx] = array.data[idx];

	}


	void RosClass::reflexValueCallback(const std_msgs::Float32MultiArray array){
		assert(array.data.size()==motor_num);
		for(uint8_t idx=0;idx<motor_num;idx++)
			reflexValue[idx]=array.data[idx];

	}

	/**  
	 *   @brief      the callback of receive motor data function
Parameters:
	 *   @param      array    [in]the data receive from the locomotion controller for each joint 
	 *   @return     none
	 */
	void RosClass::modelValueCallback(const std_msgs::Float32MultiArray array){
		assert(array.data.size()==motor_num);
		for(uint8_t idx=0;idx<motor_num;idx++)
		{
			modelValue[idx]=array.data[idx];
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
		terminate=(bool)termNode.data;
	}

	/**  
	 *   @brief      update motor value
Parameters:
	 *   @return     none
	 */
	void RosClass::updateMotorValue(){
		for(uint8_t idx=0;idx<motor_num;idx++)
			motorValue[idx] = modelValue[idx];//cpgValue[idx];//reflexValue[idx];// + cpgValue[idx];
		//motorValue[idx]=reflexValue[idx];// + cpgValue[idx];
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
		if((deltaTime > 3*1000000/ros_rate) && !firstFlag)
		{
			printf("delta:%ld, internal:%.3f, ros_rate:%d\n", deltaTime, (double)(3*1000000/ros_rate), ros_rate);
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
