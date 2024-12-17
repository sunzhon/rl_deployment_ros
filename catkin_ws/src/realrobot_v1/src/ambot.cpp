#include "ambot.h"
#include "unistd.h"
#include "assert.h"
#include <sys/time.h>
#include <iostream>
#include <vector>
#include <utility>
#define PI 3.1415926
#include <Eigen/Dense>
using namespace Eigen;

// bool connect_eboard=false;

namespace robot_ns{
	/*
	 * Motor pos limitation, the index of the array are motor_id
	 */
	const float min_motor_limit[] = { 0.0,//ID=0
		-0.69, -1.57,//ID=1,2
		-0.78, -1.57, -2.18,//ID=3,4,5
		-0.78, -0.61, -0.52,//ID=6,7,8
		-0.78, -1.57, -2.18,//ID=9,10,11
		-0.78, -0.61, -0.52,//ID=12,13,14
		-1.57, -1.57, -1.57,
		-1.57, -1.57, -1.57
	};

	const float max_motor_limit[] = { 0.0, // ID=0 
		1.22, 0.87, //ID=1,2
		0.78, 0.61, 0.52,// ID=3,4,5
		0.78, 1.57, 2.18,
		0.78, 0.61, 0.52,
		0.78, 1.57, 2.18,
		1.57, 1.57, 1.57,
		1.57, 1.57, 1.57
	};

	float ByteToFloat(unsigned char* byteArry)
	{
		return *((float*)byteArry);
	}

	// the motor idx of joints from FL leg joint1 to HR joint3
	// the index of the motor in a seq list, rather than their ID
	// for example, 
	const int action_map[12] ={2,3,4, 5,6,7,  8,9,10,  11,12,13};

	Ambot::Ambot(){
		/**
		 *@Descrption: Ambot class construct function
		 *@Objects: Dynamixelworkbence, serial_imu, serial_forceBoard
		 * */
#ifdef DYNAMIXEL_MODE_WORKBENCH
		dxl_wb = new DynamixelWorkbench;
#else
		dxl_wb = new DynamixelClass;
#endif

		imuConnectFlag = false;
		imuSensor = new imu_vn_100::ImuVn100();
		forceSensor =new bota_ns::BotaSensor();

		ROS_INFO("Real robot init successfully!");
	}

	Ambot::~Ambot(){
		for (uint8_t index = 0; index < dxl_cnt; index++)
			dxl_wb->itemWrite(dxl_id[index], "Torque_Enable", 0);
		delete dxl_wb;
		// ser_imu->close();
		// delete ser_imu;
		// ser_eboard->close();
		// delete ser_eboard;
		delete imuSensor;
		delete forceSensor;
	}

	void Ambot::initMsg(){
		printf("-----------------------------------------------------------------------\n");
		printf("  There are these Dynamixel motors             \n");
		printf("-----------------------------------------------------------------------\n");
		printf("\n");

		if(dxl_cnt==0){
			ROS_INFO("DIDN't FIND MOTOR !");
			ROS_ERROR("NO MOTORS!");
		}
		assert(dxl_cnt > 0 );   //make sure robot have at least one motor
		//make sure dxl_cnt less than MAX_NUMBER_MOTOR,this make sure no bug
		assert(dxl_cnt < MAX_NUMBER_MOTOR);

		for (int index = 0; index < dxl_cnt; index++)
		{
			printf("MODEL   : %s\n", dxl_wb->getModelName(dxl_id[index]));
			printf("ID      : %d\n", dxl_id[index]);
			printf("dxl_cnt : %d\n",dxl_cnt);
			printf("\n");
		}
		printf("-----------------------------------------------------------------------\n");
	}

	bool Ambot::set_motors(){
		//configure motors
		const char *log;
		static int32_t motorState = 0;
		bool result;
		for (int index = 0; index < dxl_cnt; index++){
			//read motor state 
			result = dxl_wb->itemRead(dxl_id[index], "Hardware_Error_Status", &motorState, &log);
			if (result == false)
			{
				printf("%s\n", log);
				printf("Motor: %d\n", dxl_id[index]);
				perror("Failed to read state of  Dynamixel motor\n");
				return false;
			}
			//reboot the motor if motor state not equal 0
			if(motorState != 0)
			{
				result = dxl_wb->reboot(dxl_id[index], &log);
				if (result == false)
				{
					printf("%s\n", log);
					printf("Motor: %d\n", dxl_id[index]);
					perror("Failed to reboot  Dynamixel motor\n");
					return false;
				}
			}

			//torque disable
			result = dxl_wb->torqueOff(dxl_id[index],&log);// torque off
			if (result == false)
			{
				printf("%s\n", log);
				printf("Motor: %d\n", dxl_id[index]);
				perror("Failed to torque off Dynamixel motor\n");
				return false;
			}
			//set motor return delay time
			result = dxl_wb->itemWrite(dxl_id[index],"Return_Delay_Time", return_delay_time, &log);
			if (result == false)
			{
				printf("%s\n", log);
				printf("Motor: %d\n", dxl_id[index]);
				perror("Failed to set motor return delay time\n");
				return false;
			}
			//set min angle limit
			result = dxl_wb->itemWrite(dxl_id[index],"Min_Position_Limit", 
					angle2Position(min_motor_limit[dxl_id[index]],true), &log);
			if (result == false)
			{
				printf("%s\n", log);
				printf("Motor: %d\n", dxl_id[index]);
				perror("Failed to set min angle to Dynamixel motor\n");
				return false;
			}

			//set max angle limit
			result = dxl_wb->itemWrite(dxl_id[index],"Max_Position_Limit", 
					angle2Position(max_motor_limit[dxl_id[index]], true), &log);
			if (result == false)
			{
				printf("%s\n", log);
				printf("Motor: %d\n", dxl_id[index]);
				perror("Failed to set max angle to Dynamixel motor\n");
				return false;
			}

			//set control mode according to the motor model
			string model="MX-28-2";
			if(model.compare(dxl_wb->getModelName(dxl_id[index]))==0){
				result = dxl_wb->setPositionControlMode(dxl_id[index],&log);

				if (result == false)
				{
					printf("%s\n", log);
					printf("Motor: %d\n", dxl_id[index]);
					perror("Failed to initialize Dynamixel motor\n");
					return false;
				}
			}
			else{
				// result = dxl_wb->setCurrentBasedPositionControlMode(dxl_id[index],&log);
				result = dxl_wb->setPositionControlMode(dxl_id[index],&log);

				if (result == false)
				{
					printf("%s\n", log);
					printf("Motor: %d\n", dxl_id[index]);
					perror("Failed to initialize Dynamixel motor\n");
					return false;
				}
			}
			//set move velocity of profile to motor
			result = dxl_wb->itemWrite(dxl_id[index],"Profile_Velocity", profile_velocity,&log);
			if (result == false)
			{
				printf("%s\n", log);
				printf("Motor: %d\n", dxl_id[index]);
				perror("Failed to set profile of velocity to Dynamixel motor\n");
				return false;
			}
			//set move acceleration of profile to motor
			result = dxl_wb->itemWrite(dxl_id[index],"Profile_Acceleration",profile_acceleration,&log);
			if (result == false)
			{
				printf("%s\n", log);
				printf("Motor: %d\n", dxl_id[index]);
				perror("Failed to initialize Dynamixel motor\n");
				return false;
			}
			//enable motor 
			result = dxl_wb->torqueOn(dxl_id[index],&log);
			//result = dxl_wb->torqueOff(dxl_id[index],&log);
			if (result == false)
			{
				printf("%s\n", log);
				printf("Motor: %d\n", dxl_id[index]);
				perror("Failed to torque on Dynamixel motor\n");
				return false;
			}

		}

		int index=0;
		result = dxl_wb->addSyncWriteHandler(dxl_id[index], "Goal_Position", &log);
		if (result == false)
		{
			printf("%s\n", log);
			printf("Motor: %d\n", dxl_id[index]);
			perror("Failed to add sync write handler: Goal_Position\n");
			return false;
		}

		result = dxl_wb->addSyncWriteHandler(dxl_id[index], "Goal_Current", &log);
		if (result == false)
		{
			printf("%s\n", log);
			printf("Motor: %d\n", dxl_id[index]);
			perror("Failed to add sync write handler: Goal_Current\n");
			return false;
		}

		result = dxl_wb->addSyncReadHandler(dxl_id[index], "Present_Position", &log);
		if (result == false)
		{
			printf("%s\n", log);
			printf("Motor: %d\n", dxl_id[index]);
			perror("Failed to add sync read handler position\n");
			return false;
		}

		result = dxl_wb->addSyncReadHandler(dxl_id[index], "Present_Velocity", &log);
		if (result == false)
		{
			printf("%s\n", log);
			printf("Motor: %d\n", dxl_id[index]);
			perror("Failed to add sync read handler velocity\n");
			return false;
		}

		result = dxl_wb->addSyncReadHandler(dxl_id[index], "Present_Current", &log);
		if (result == false)
		{
			printf("%s\n", log);
			printf("Motor: %d\n", dxl_id[index]);
			perror("Failed to add sync read handler current\n");
			return false;
		}

		result = dxl_wb->addSyncReadHandler(dxl_id[index], "Present_Input_Voltage", &log);
		if (result == false)
		{
			printf("%s\n", log);
			printf("Motor: %d\n", dxl_id[index]);
			perror("Failed to add sync read handler voltage\n");
			return false;
		}
		return true;
	}


	bool Ambot::init(map<string, float>& robot_params, map<string,string>& robot_devices){

		//0) loading rosparam parameters and init dynamixel
		string device_key="ambot_devices";
		string param_key="ambot_params";

		//0.1 dynamixel motor
		string dxl_device = (string)robot_devices[device_key+string("/dxl_device")];
		int dxl_baud  = (int)(robot_params[param_key+string("/dxl_baud")]);
		int scan_range  = (int)(robot_params[param_key+string("/scan_range")]);
		profile_velocity = (int)(robot_params[param_key+string("/profile_velocity")]);
		profile_acceleration = (int)(robot_params[param_key+string("/profile_acceleration")]);
		return_delay_time = (int)(robot_params[param_key+string("/return_delay_time")]);


		//0.2 many num of limbs
		leg_num = (int)(robot_params[param_key+string("/leg_num")]);
		motor_num = (int)(robot_params[param_key+string("/motor_num")]);
		action_num = (int)(robot_params[param_key+string("/action_num")]);
		sensor_num = (int)(robot_params[param_key+string("/sensor_num")]);
		pose_num = (int)(robot_params[param_key+string("/pose_num")]);
		force_data_num = (int)(robot_params[param_key + string("/force_data_num")]);

		//0.3 imu
		string imu_device = (string)robot_devices[device_key+string("/imu_device")];
		int imu_baud  = (int)(robot_params[param_key+string("/imu_baud")]);

		//0.4 bota force sensor 
		std::vector<std::string> force_device;
		force_device.push_back((string)robot_devices[device_key+string("/force_device_FL")]);
		force_device.push_back((string)robot_devices[device_key+string("/force_device_FR")]);
		force_device.push_back((string)robot_devices[device_key+string("/force_device_HL")]);
		force_device.push_back((string)robot_devices[device_key+string("/force_device_HR")]);
		int force_baud= (int)(robot_params[param_key+string("/force_baud")]);


		//1 Open MX serial motors using U2D2
		const char *log;
		bool result;
		if(access(dxl_device.c_str(),0)==F_OK){// whether dynamixel motors specified in yaml file exist
			//if(strcmp(dxl_device.c_str(),"/dev/XM430_D2D")==0){
			result=dxl_wb->init(dxl_device.c_str(), dxl_baud, &log);
			if(result==false){
				printf("%s\n",log);
				perror("Fail to init Dynamixel motors!\n");
				return false;
			}
			if (dxl_wb->scan(dxl_id, &dxl_cnt, scan_range, &log) != true) {
				ROS_ERROR("Not found Motors, Please check scan range or baud rate");
				ros::shutdown();
				return false;
			}else{
				ROS_INFO("Find Dynamixel device successfully!");
				printf("dynamixel motor number: %i\n", dxl_cnt);
			}
		}else if(access("/dev/U2D2",0)==F_OK){// whether force board exist
			result=dxl_wb->init("/dev/UD2D", dxl_baud, &log);
			if(result==false){
				printf("%s\n",log);
				perror("Fail to init Dynamixel motors!\n");
				return false;
			}
			if (dxl_wb->scan(dxl_id, &dxl_cnt, scan_range, &log) != true) {
				ROS_ERROR("Not found Motors, Please check scan range or baud rate");
				ros::shutdown();
				return false;
			}
			ROS_INFO(" Motor driver is U2D2 \n");
		}

		// display motor information
		initMsg(); 

		//2) setup motors
		set_motors();

		//3 open force sensor according to exist situation
		for (int i = 0; i < force_device.size(); i++)
		{
			//make sure all sensor devices exist
			if (access(force_device.at(i).c_str(),0)==F_OK)
			{
				printf("device:%s exist!\n", force_device.at(i).c_str());
			}else{
				printf("device:%s doesn't exist!\n", force_device.at(i).c_str());
				break;
			}
			//initial force sensor if all sensor port exist
			if((force_device.size() - 1)  == i)
			{
				forceSensor->initSensor(force_device, force_baud);
				printf("initial force sensor\n\n\n");
				forceConnectFlag = forceSensor->getSensorInitState();
			}

		}

		//4) open and init pose imu sensor
		//whether imu sensor port exist
		if(access(imu_device.c_str(),0)==F_OK)
		{
			if(imuSensor->Initialize(imu_device, imu_baud))
			{
				imuConnectFlag = true;
				imuSensor->Stream(true);
			}else{
				ROS_WARN("IMU initial failed!"); 
			}
		}
		else{
			ROS_WARN("Did not find IMU device");
		}


		//5) Variable initialization
		//i) local control
		position_error.resize(dxl_cnt);
		float_position_error.resize(dxl_cnt);
		float_velocity_error.resize(dxl_cnt);
		previous_goal_position.resize(dxl_cnt);

		// //ii) motor commands from controller
		goal_position.resize(dxl_cnt);
		goal_velocity.resize(dxl_cnt);
		goal_current.resize(dxl_cnt);

		//iii) sensory feedback of motors
		present_position.resize(motor_num);
		present_velocity.resize(motor_num);
		present_current.resize(motor_num);
		present_voltage.resize(motor_num);
		physical_present_position.resize(motor_num);
		physical_present_velocity.resize(motor_num);
		physical_present_current.resize(motor_num);
		physical_present_voltage.resize(motor_num);

		//iv) sensory feedback of force sensors
		grf.resize(leg_num*force_data_num);

		//v) sensory feedback of imu
		pose.resize(pose_num);

		//vi) storage of motor commands and all sensory feedback signals
		motorValue.resize(action_num);
		sensorValue.resize(sensor_num);

		//vii) wait
		ros::Duration(3.5).sleep();
		p_gain=500;


		ROS_INFO("Ambot init done!");
		return true;
		}


		sensor_msgs::Imu imuData;

		void Ambot::getSensoryValue(ambot_msgs::RobotState &value){
			assert(value.motorState.size()==action_num);

			// Read sensory signals from sensors
			//readFootForce();
			readJoints();

			try
			{
				imuData = imuSensor->getVn100ImuData();
			}catch(...)
			{
				ROS_WARN("read imu failed!");
			}


			try
			{
				uint8_t temp_idx=0;
				// Save all sensory signals into value variables
				for(uint8_t idx=0; idx < action_num; idx++){ //12 joint positions
					temp_idx = action_map[idx];
					value.motorState[idx].pos = physical_present_position.at(temp_idx);
					value.motorState[idx].vel = physical_present_velocity.at(temp_idx);
					value.motorState[idx].cur = physical_present_current.at(temp_idx);
					value.motorState[idx].vol = physical_present_voltage.at(temp_idx);
				}


				// imu info
				value.imu.acceleration.x= imuData.linear_acceleration.x;
				value.imu.acceleration.y= imuData.linear_acceleration.y;
				value.imu.acceleration.z= imuData.linear_acceleration.z;

				value.imu.gyroscope.x = imuData.angular_velocity.x;
				value.imu.gyroscope.y = imuData.angular_velocity.y;
				value.imu.gyroscope.z = imuData.angular_velocity.z;

				float imu_quaternion_x = imuData.orientation.x;
				float imu_quaternion_y = imuData.orientation.y;
				float imu_quaternion_z = imuData.orientation.z;
				float imu_quaternion_w = imuData.orientation.w;
				Eigen::AngleAxisd reference_frame_axis_angle(-M_PI, Vector3d(1, 0, 0));
				Quaterniond refer_q(reference_frame_axis_angle);

				Quaterniond imu_q(imu_quaternion_w, imu_quaternion_x,
						imu_quaternion_y,imu_quaternion_z);
				Quaterniond new_imu_q =  refer_q * imu_q;

				//cout << "new imu q" << endl << new_imu_q.coeffs() << endl;
				//cout << "old imu eular:" << endl << imu_q.matrix().eulerAngles(0,1,2)*57 << endl;
				//cout << "new imu eular:" << endl << new_imu_q.matrix().eulerAngles(0,1,2)*57 << endl;


				value.imu.quaternion.x = new_imu_q.x();
				value.imu.quaternion.y = new_imu_q.y();
				value.imu.quaternion.z = new_imu_q.z();
				value.imu.quaternion.w = new_imu_q.w();



				//value.imu.quaternion.x = imuData.orientation.x;
				//value.imu.quaternion.y = imuData.orientation.y;
				//value.imu.quaternion.z = imuData.orientation.z;
				//value.imu.quaternion.w = imuData.orientation.w;

			}
			catch(...){
				ROS_WARN("copy sensor value error!");

			}
		}




		void Ambot::setMotorValue(const ambot_msgs::RobotAction& value){
			vector<float> tmp;
			tmp.resize(motor_num);
			uint8_t temp_idx=0;
			if(value.motorAction.size()>0){
				for(int idx=0; idx<value.motorAction.size(); idx++){
					temp_idx = action_map[idx];
					tmp.at(temp_idx) = value.motorAction[idx].q;
				}
			}

			value2Position(tmp, goal_position);
			localController();
		}



		void Ambot::readJoints(){
			//1) read joint angles
			const uint8_t handler_index_read_position=0;
			const uint8_t handler_index_read_velocity=1;
			const uint8_t handler_index_read_current=2;
			const uint8_t handler_index_read_voltage=3;
			bool result;
			const char * log;
			// read joint position
			result = dxl_wb->syncRead(handler_index_read_position, &log);
			if (result == false)
			{   
				printf("%s\n", log);
				printf("Failed to sync read position\n");
			}   
			result = dxl_wb->getSyncReadData(handler_index_read_position, &present_position[0], &log);

			// read joint velocity
			result = dxl_wb->syncRead(handler_index_read_velocity, &log);
			if (result == false)
			{   
				printf("%s\n", log);
				printf("Failed to sync read velocity\n");
			}   
			result = dxl_wb->getSyncReadData(handler_index_read_velocity, &present_velocity[0], &log);

			// read joint current
			result = dxl_wb->syncRead(handler_index_read_current, &log);
			if (result == false)
			{   
				printf("%s\n", log);
				printf("Failed to sync read current\n");
			}   
			result = dxl_wb->getSyncReadData(handler_index_read_current, &present_current[0], &log);

			// read joint voltage
			result = dxl_wb->syncRead(handler_index_read_voltage, &log);
			if (result == false)
			{   
				printf("%s\n", log);
				printf("Failed to sync read voltage\n");
			}
			result = dxl_wb->getSyncReadData(handler_index_read_voltage, &present_voltage[0], &log);

			for(int idx=0; idx < dxl_cnt; idx++){
				// Convert the int into physical values
				physical_present_position.at(idx) = dxl_wb->convertValue2Radian(dxl_id[idx], present_position[idx]);//output value unit is radian
				physical_present_velocity.at(idx) = dxl_wb->convertValue2Velocity(dxl_id[idx],present_velocity[idx]) *2.0*PI/60.0;//unit is rpm, and change it to rad/s,
				physical_present_current.at(idx) = dxl_wb->convertValue2Current(present_current[idx]);// output value unit is mA/ Nm
				physical_present_voltage.at(idx) = 0.1*present_voltage[idx]; // convert output value unit to be  V
			}

		}

		void Ambot::readImu(){
			if (imuConnectFlag)
			{
				sensor_msgs::Imu imuData = imuSensor->getVn100ImuData();
				pose.at(0) = imuData.linear_acceleration.x;
				pose.at(1) = imuData.linear_acceleration.y;
				pose.at(2) = imuData.linear_acceleration.z;

				pose.at(3) = imuData.angular_velocity.x;
				pose.at(4) = imuData.angular_velocity.y;
				pose.at(5) = imuData.angular_velocity.z;

				pose.at(6) = imuData.orientation.x;
				pose.at(7) = imuData.orientation.y;
				pose.at(8) = imuData.orientation.z;
				pose.at(9) = imuData.orientation.w;
			}

		}

		void Ambot::readFootForce(){
			/**
			 *@Description: Read force data from force sensor-V2 or knne joint motor torque -V1
			 *
			 *
			 * */
			if (forceConnectFlag)
			{
				forceSensor->getSensorData(grf);
			}

		}



		void Ambot::localController(){
			for(uint8_t index =0; index < dxl_cnt;index++){
				//goal_current[index] = p_gain*200 > 400 ? 400 : p_gain*200;
				goal_current[index] = 20000; //p_gain > 600 ? 600: p_gain;
			}
			writeServoValue();
		}

		void Ambot::getParameters(){


		}

		void Ambot::writeServoValue(){
			const uint8_t handler_index_write_position = 0;
			const uint8_t handler_index_write_current = 1;
			bool result;
			const char * log;

			result = dxl_wb->syncWrite(handler_index_write_position, &goal_position[0], &log);
			if (result == false)
			{
				printf("%s\n", log);
				printf("Failed to sync write position\n");
			}

			// result = dxl_wb->syncWrite(handler_index_write_current, &goal_current[0], &log);
			// if (result == false)
			// {
			//     printf("%s\n", log);
			//     printf("Failed to sync write current\n");
			// }

		}

		void Ambot::value2Position(const std::vector<float>& value, std::vector<int32_t>& position){
			assert(value.size()>=dxl_cnt);
			assert(position.size()>=dxl_cnt);
			for(int index=0; index<dxl_cnt; index++){
				position[index] = angle2Position(value.at(index),true);
			}
		}


		/**
		 * @brief angle2Position
		 * Converts angle input into position data (based on motor model provided) code only for 1 model
		 * @param angle Angle to be converted
		 * @param Motor model, as encoded on Dynamixel motor's control table
		 * @param isRadians Boolean flag for unit used in expressing the angle, true if radians (radians by default)
		 * @return Position value corresponding to the joint angle (mid-position is 0 degrees/rad)
		 */
		int angle2Position(double angle, bool isRadians)
		{
			int position=2048;
			double angleInDegree = isRadians ? angle*180/PI : angle;
			int Model_max_position=4095;
			int range=360;
			position = (angleInDegree + range/2) * (double)Model_max_position/range + 0.5;
			if (position > Model_max_position)
				position = Model_max_position;
			else if (position < 0)
				position = 0;    	

			return position;
		}



		/**  
		 *   @brief      
		 *   @param      data    [in]the data quote of need to log
		 *   @return     none
		 */
		void Ambot::storeData(std::vector<std::pair<string, float>>& data) 
		{
			data.clear();
			for(uint8_t idx=0; idx < action_num; idx++){ 
				data.push_back(pair<string, float>("jcm_"+to_string(idx), 0.0));
			}
			// joint feedbacks
			for(uint8_t idx=0; idx < dxl_cnt; idx++){ 
				data.push_back(pair<string, float>("jointPosition_"+to_string(idx), physical_present_position.at(idx)));
			}
			for(uint8_t idx=0; idx < dxl_cnt; idx++){ 
				data.push_back(pair<string, float>("jointVelocity_"+to_string(idx), physical_present_velocity.at(idx)));
			}
			for(uint8_t idx=0; idx < dxl_cnt; idx++){ 
				data.push_back(pair<string, float>("jointCurrent_"+to_string(idx), physical_present_current.at(idx)));
			}
			for(uint8_t idx=0; idx < action_num; idx++){ 
				data.push_back(pair<string, float>("jointVoltage_"+to_string(idx), physical_present_voltage.at(idx)));
			}
			data.push_back(pair<string, float>("imu_accel_x", imuData.linear_acceleration.x));
			data.push_back(pair<string, float>("imu_accel_y", imuData.linear_acceleration.y));
			data.push_back(pair<string, float>("imu_accel_z", imuData.linear_acceleration.z));

			data.push_back(pair<string, float>("imu_ang_x", imuData.angular_velocity.x));
			data.push_back(pair<string, float>("imu_ang_y", imuData.angular_velocity.y));
			data.push_back(pair<string, float>("imu_ang_z", imuData.angular_velocity.z));

			data.push_back(pair<string, float>("imu_orienta_x", imuData.orientation.x));
			data.push_back(pair<string, float>("imu_orienta_y", imuData.orientation.y));
			data.push_back(pair<string, float>("imu_orienta_z", imuData.orientation.z));
			data.push_back(pair<string, float>("imu_orienta_w", imuData.orientation.w));


		}




	}
