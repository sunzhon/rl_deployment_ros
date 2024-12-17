#ifndef __DYNAMIXEL_CLASS_H__
#define __DYNAMIXEL_CLASS_H__

//enable print debug information
// #define PRINT_TIME
//choose use dynamixelSDK or workbench as the low driver
// #define DYNAMIXEL_MODE_WORKBENCH

#ifdef DYNAMIXEL_MODE_WORKBENCH

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_controllers/dynamixel_workbench_controllers.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

class DynamixelClass:public DynamixelWorkbench
{
  DynamixelClass();
  ~DynamixelClass();
};
#else

#include "dynamixel_sdk.h" 
#include "iostream"
#include <string.h>
#include "unistd.h"

typedef struct 
{
  const char *item_name;
  uint16_t    address;
  uint8_t	    item_name_length;
  uint16_t    data_length;
} ControlItem;

typedef struct 
{
  ControlItem *control_item; 
  dynamixel::GroupSyncWrite *groupSyncWrite;    
} SyncWriteHandler;

typedef struct 
{
  uint16_t    address;
  uint16_t    data_length;
  dynamixel::GroupSyncRead  *groupSyncRead;     
} SyncReadHandler;

#define MAX_HANDLER_NUM 5
#define MAX_ID_NUM      30
class DynamixelClass
{   
  private:
    SyncWriteHandler syncWriteHandler_[MAX_HANDLER_NUM];
    SyncReadHandler  syncReadHandler_[MAX_HANDLER_NUM];
    dynamixel::PortHandler*   portHandler_;
    dynamixel::PacketHandler* packetHandler_;
  public:
    DynamixelClass();
    ~DynamixelClass();

    bool setPortHandler(const char *device_name, const char **log);
    bool setBaudrate(uint32_t baud_rate, const char **log);
    bool setPacketHandler(float protocol_version, const char **log);
    bool scan(uint8_t *get_id,
        uint8_t *get_the_number_of_id, 
        uint8_t range = 253,
        const char **log = NULL);

    bool scan(uint8_t *get_id,
        uint8_t *get_the_number_of_id, 
        uint8_t start_number,
        uint8_t end_number,
        const char **log = NULL);
    bool reboot(uint8_t id, const char **log = NULL);
    bool init(const char *device_name = "/dev/ttyUSB0", uint32_t baud_rate = 57600U, const char **log = (const char **)NULL);
    bool itemWrite(uint8_t id, const char *item_name, int32_t data, const char **log = NULL);
    bool itemRead(uint8_t id, const char *item_name, int32_t *data, const char **log = NULL);
    const char * getModelName(uint8_t id, const char **log = NULL);
    
    bool torque(uint8_t id, int32_t onoff, const char **log = NULL);
    bool torqueOn(uint8_t id, const char **log = NULL);
    bool torqueOff(uint8_t id, const char **log = NULL);

    bool setOperatingMode(uint8_t id, const uint8_t model);
    bool setPositionControlMode(uint8_t id, const char **log);
    bool setCurrentBasedPositionControlMode(uint8_t id, const char **log);
    bool addSyncWriteHandler(uint8_t id, const char *item_name, const char **log);
    bool addSyncReadHandler(uint8_t id, const char *item_name, const char **log);
    void getParam(int32_t data, uint8_t *param);
    bool syncWrite(uint8_t index, int32_t *data, const char **log);
    bool syncRead(uint8_t index, const char **log);
    bool getSyncReadData(uint8_t index, int32_t *data, const char **log);

    float getProtocolVersion(void);
    int16_t convertCurrent2Value(uint8_t id, float current);
    float convertValue2Radian(uint8_t id, int32_t value);
    float convertValue2Velocity(uint8_t id, int32_t value);
    float convertValue2Current(int16_t value);
    bool write_register(uint8_t id, uint16_t address, uint16_t length, int32_t data);
    bool read_register(uint8_t id, uint16_t address, uint16_t length, int32_t *data);

  private:
    //save address of attribute
    std::map<std::string, int> name2addr;
    //save length of attribute
    std::map<std::string, int> name2length;
    //save model of motor
    std::map<int, int> id_model;
    //save motor id
    uint8_t dxl_id[MAX_ID_NUM];
    //save number of motor
    uint8_t dxl_cnt;
    //save number of sync write handle
    uint8_t sync_write_handler_cnt_;
    //save number of sync read handle
    uint8_t sync_read_handler_cnt_;

};

typedef struct
{
  int dxl_comm_result;
  bool dxl_addparam_result;
  bool dxl_getdata_result;
  uint8_t dxl_error;
} ErrorFromSDK;

#endif

#endif