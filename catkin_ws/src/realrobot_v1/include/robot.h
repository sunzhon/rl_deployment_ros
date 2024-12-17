#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "ambot.h"
#include "rosClass.h"
#include "ambot_msgs/RobotState.h"
#include "ambot_msgs/RobotAction.h"
#include <pthread.h>
#include <errno.h>
#include <mutex>
#include "Log.h"

using namespace robot_ros_ns;
namespace robot_ns{
    enum RealRobotSyncState
    {
        NOT_SYNC_STATE = 0,
        SYNC_PERIOD_ONE_STATE,
        SYNC_PERIOD_TWO_STATE,
    };

    class Robot{
        public:
            Robot(int argc, char** argv);
            ~Robot();
            bool run();
        private:
            int syncState;
            int controllerRate;
            map<string, string> robot_devices;
            map<string, float> robot_params;
	    ambot_msgs::RobotState sensorValue;
	    ambot_msgs::RobotAction motorValue;
            RosClass* ros;
            Ambot* rob;

        private:
            pthread_t tid;
            static void *paramServiceThread(void *arg);
            void paramService();
            //log system
            unsigned int files_num;
            std::vector<std::string> files;
            stcontroller::Log* log;           

	private:
            std::vector<std::pair<string, float>> experiment_data;

    };


}


#endif
