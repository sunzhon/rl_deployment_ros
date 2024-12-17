#ifndef __STRUCT_H__
#define __STRUCT_H__
#include <iostream>
#define MY_PI 3.141592654
typedef struct 
{
    std::string motorDevName;
    std::string sensorDevName;
    unsigned int motorDevBaud;
    unsigned int sensorDevBaud;
    float motorMaxAngle;
    float motorMinAngle;
    float motorVelocity;
    uint8_t motorControlMode;
    uint8_t limbNum;
    uint8_t legNum;
    uint8_t motorNum;
    uint8_t imuNum;
    uint8_t forceDataNum;
    uint8_t sensorNum;
    uint16_t rosRate;
}AmbotDeviceTPDF;


#endif