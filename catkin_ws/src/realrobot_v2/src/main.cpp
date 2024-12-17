#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "robot.hpp"
#include <sys/time.h>

/*register signal handler to deal ctrl+C, need a class pointer*/
Robot* myRobot;

// add get interrupt signal of SIGINT
void signalHandler(int signum) {
    myRobot->setThreadRunFlag();
    usleep(200000);
    std::cout << "Received signal: " << signum << ", Ambot program will exit()." << std::endl;
    // 调用类的成员函数
    myRobot->runEnd();
    exit(0);
}

using namespace std;
int main(int argc, char** argv)
{
    Robot robot(argc, argv);
    if(robot.init())
    {
        //2 register exception signal get handler 
        myRobot = &robot;
        signal(SIGINT, signalHandler);
        while (1)
        {
            robot.run();
        }
    }
    return 0;

}
