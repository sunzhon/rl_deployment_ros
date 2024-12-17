#include "robot.h"

using namespace robot_ros_ns;
std::mutex paramlock;
namespace robot_ns
{
    Robot::Robot(int argc, char** argv)
    {
	// instancetialize ros interface and specific robot objects
        ros = new RosClass(argc,argv);
        rob = new Ambot();

        // load param from ros server
        ros->getParameters(robot_params, robot_devices);
        
        // init robot
        if(!rob->init(robot_params, robot_devices)){
            perror("Robot init false!\n");
        }
        
        // init variables 
        printf("Action number is %i!\n", rob->action_num);
        motorValue.motorAction.resize(rob->action_num);
        sensorValue.motorState.resize(rob->action_num);
        // get parameter service thread
        if(pthread_create(&tid, NULL, paramServiceThread, (void *)this) != 0){ 
            perror("Create thread fail in controller!\n");
        } 
        
        // create log files
        string str[]={"controlfile_data"};
        vector<string> strfiles(str,str+sizeof(str)/sizeof(str[0]));
        files=strfiles;
        files_num=files.size();
        log= new stcontroller::Log(files);

        rob->getParameters();

        ROS_INFO("Robot node start successful!\n");


    }

    Robot::~Robot(){
        robot_params.clear();
        robot_devices.clear();
        delete rob; //robot
        delete ros; //ros interface
	delete log;
    }

    void Robot::paramService(){
        rob->getParameters();
    }   

    void *Robot::paramServiceThread(void * arg){
        Robot * ptr = (Robot*) arg;
        while(!ptr->ros->terminate){
        ptr->paramService();
        sleep(1);
        }
        pthread_exit(0);
    }

    bool Robot::run(){
        if(ros::ok())
        {
            if(ros->terminate)
            {
                ros->setTerminateValue();
                return false;                   //if return false, program will end, then will go to deconstruct function
            }

            rob->getSensoryValue(sensorValue); // get sensor value from robots
            ros->pubSensorValue(sensorValue); // pub sensor value

            ros->fetchMotorValue(motorValue);   // fetch motor values from topics
            if(ros->recive_actionvalue_flag)
                rob->setMotorValue(motorValue);     // set motor value (cmd) to a robot
            
            //save data to log
            // get data from contol 
            experiment_data.clear();
            rob->storeData(experiment_data);//with data name
            // save dataset in csv files
            log->saveData(files.at(0), experiment_data);
            log->step();
            
            ros->rosSleep();
            return true;
        }else{
            return false;
        }
    }
}
