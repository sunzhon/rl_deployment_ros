#include "dynamixelClass.h"
#include "assert.h"

#ifdef DYNAMIXEL_MODE_WORKBENCH
DynamixelClass::DynamixelClass(){}
DynamixelClass::~DynamixelClass(){}

#else
DynamixelClass::DynamixelClass(){}
DynamixelClass::~DynamixelClass(){}

static const uint8_t CURRENT_CONTROL_MODE                  = 0;
static const uint8_t VELOCITY_CONTROL_MODE                 = 1;
static const uint8_t POSITION_CONTROL_MODE                 = 3;
static const uint8_t EXTENDED_POSITION_CONTROL_MODE        = 4;
static const uint8_t CURRENT_BASED_POSITION_CONTROL_MODE   = 5;
static const uint8_t PWM_CONTROL_MODE                      = 16;
static const uint8_t TORQUE_CONTROL_MODE                   = 100;
static const uint8_t MULTI_TURN_MODE                       = 101;    

/**  
*   @brief      read data function
    Parameters:
*   @param      id      [in]the servo id 
*   @param      address [in]the data address
*   @param      length  [in]the data length
*   @param      data    [out]the data pointer
*   @return     true:read successful; false:read failed    
    */
bool DynamixelClass::read_register(uint8_t id, uint16_t address, uint16_t length, int32_t *data)
{
    int dxlCommResult = COMM_TX_FAIL;               // Communication result
    uint8_t dxlError = 0;                            // Dynamixel error

    uint8_t data_1_byte  = 0;
    uint16_t data_2_byte = 0;
    uint32_t data_4_byte = 0;


    if (length == 1) {
        dxlCommResult = packetHandler_->read1ByteTxRx(portHandler_,
                            id, address, &data_1_byte, &dxlError);
    }
    else if (length == 2) {
        dxlCommResult = packetHandler_->read2ByteTxRx(portHandler_,
                            id, address, &data_2_byte, &dxlError);
    }
    else if (length == 4) {
        dxlCommResult = packetHandler_->read4ByteTxRx(portHandler_,
                            id, address, &data_4_byte, &dxlError);
    }
    else
        std::cout << "Check data length" << std::endl;

    if (dxlCommResult != COMM_SUCCESS) {
        std::cout << packetHandler_->getTxRxResult(dxlCommResult) << std::endl;
        return false;
    }
    else if (dxlError != 0) {
        std::cout << packetHandler_->getTxRxResult(dxlError) << std::endl;
        return false;
    }
    else {
        if (length == 1)
            *data = data_1_byte;
        else if (length == 2)
            *data = data_2_byte;
        else if (length == 4)
            *data = data_4_byte;
        else
            *data = data_1_byte;
        return true;
    }
    return false;
}

/**  
*   @brief      itemRead data function
    Parameters:
*   @param      id        [in]the servo id 
*   @param      item_name [in]the name of send data
*   @param      data      [in]data waiting for send
*   @param      log       [in]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::itemRead(uint8_t id, const char *item_name, int32_t *data, const char **log)
{
  //1.get the iterator of name2addr,if get it,you can get name2length too.
  auto iter = name2addr.find(item_name);
  if (iter != name2addr.end())
  {
    //2.send data
    read_register(id, iter->second, name2length[item_name], data);
    return true;
  }else
  {
    //3.print warning information
    std::cout <<  "itemRead can't find '" << item_name << "' in name2addr;" << std::endl;
    return false;
  }
}


/**  
*   @brief      write data function
    Parameters:
*   @param      id      [in]the servo id 
*   @param      address [in]the data address
*   @param      length  [in]the data length
*   @param      data    [in]the data
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::write_register(uint8_t id, uint16_t address, uint16_t length, int32_t data)
{
  //1.init variables
  int dxlCommResult = COMM_TX_FAIL;               // Communication result
  uint8_t dxlError = 0;                            // Dynamixel error

  //2.judge data length and send data 
  if (length == 1) 
  {
    uint8_t dataByte = (uint8_t)data;
    dxlCommResult = packetHandler_->write1ByteTxRx(portHandler_, id, address, dataByte, &dxlError);
  }
  else if (length == 2) 
  {
    uint16_t dataByte = (uint16_t)data;
    dxlCommResult = packetHandler_->write2ByteTxRx(portHandler_, id, address, dataByte, &dxlError);
  }
  else if (length == 4) 
  {
    uint32_t dataByte = (uint32_t)data;
    dxlCommResult = packetHandler_->write4ByteTxRx(portHandler_, id, address, dataByte, &dxlError);
  }
  else
  {
    std::cout<<"Check data length"<<std::endl;
  }
  //3.check the result of data send and return value
  if(dxlCommResult != COMM_SUCCESS) 
  {
    std::cout<< packetHandler_->getTxRxResult(dxlCommResult);
    return false;/* code */
  }
  else if(dxlError != 0) 
  {
    std::cout<< packetHandler_->getTxRxResult(dxlError);
    return false;
  }
  else
  {
      return true;
  }
}


/**  
*   @brief      itemWrite data function
    Parameters:
*   @param      id        [in]the servo id 
*   @param      item_name [in]the name of send data
*   @param      data      [in]data waiting for send
*   @param      log       [in]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::itemWrite(uint8_t id, const char *item_name, int32_t data, const char **log)
{
  //1.get the iterator of name2addr,if get it,you can get name2length too.
  auto iter = name2addr.find(item_name);
  if (iter != name2addr.end())
  {
    //2.send data
    write_register(id, iter->second, name2length[item_name], data);
    return true;
  }else
  {
    //3.print warning information
    std::cout <<  "itemWrite can't find '" << item_name << "' in name2addr;" << std::endl;
    return false;
  }
}

/**  
*   @brief      set communicate port handle
    Parameters:
*   @param      device_name [in]the device name
*   @param      log         [in]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::setPortHandler(const char *device_name, const char **log)
{
  //1.init the portHandle_ according to device name
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name);

  //2.check whether the port is open
  if (portHandler_->openPort())
  {
    if (log != NULL) 
    {
      *log = "[DynamixelClass] Succeeded to open the port!";
    }
    return true;
  }
  //3.add log information
  if (log != NULL) *log = "[DynamixelClass] Failed to open the port!";
  return false;
}

/**  
*   @brief      set communicate baudrate
    Parameters:
*   @param      baud_rate   [in]set the communicate baudrate
*   @param      log         [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::setBaudrate(uint32_t baud_rate, const char **log)
{
  //1.set baudrate and check whether baudrate set success
  if (portHandler_->setBaudRate((int)baud_rate))
  {
    if (log != NULL) *log = "[DynamixelClass] Succeeded to change the baudrate!";
    return true;
  }
  //2.add log 
  if (log != NULL) *log = "[DynamixelClass] Failed to change the baudrate!";
  return false;
}

/**  
*   @brief      set communicate protocol
    Parameters:
*   @param      protocol_version    [in]set the communicate protocol
*   @param      log                 [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::setPacketHandler(float protocol_version, const char **log)
{
  //1.according to protocol set packetHandler_ 
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  //2.check whether the protocol set success
  if (packetHandler_->getProtocolVersion() == protocol_version)
  {
    if (log != NULL) *log = "[DynamixelClass] Succeeded to set the protocol!";
    return true;
  }
  //3.add log
  if (log != NULL) *log = "[DynamixelClass] Failed to set the protocol!";
  return false;
}

/**  
*   @brief      init api
    Parameters:
*   @param      device_name         [in]set the communicate device name    
*   @param      protocol_version    [in]set the communicate protocol
*   @param      log                 [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::init(const char *device_name, uint32_t baud_rate, const char **log)
{
  //1.init the name2addr and name2length
  name2addr.insert(std::pair<std::string, int>("Return_Delay_Time", 9));
  name2addr.insert(std::pair<std::string, int>("Max_Position_Limit", 48));
  name2addr.insert(std::pair<std::string, int>("Min_Position_Limit", 52));
  name2addr.insert(std::pair<std::string, int>("Torque_Enable", 64));
  name2addr.insert(std::pair<std::string, int>("Hardware_Error_Status", 70));
  name2addr.insert(std::pair<std::string, int>("Goal_Current", 102));
  name2addr.insert(std::pair<std::string, int>("Goal_Position", 116));
  name2addr.insert(std::pair<std::string, int>("Present_Current", 126));
  name2addr.insert(std::pair<std::string, int>("Present_Velocity", 128)); 
  name2addr.insert(std::pair<std::string, int>("Present_Position", 132));
  name2addr.insert(std::pair<std::string, int>("Present_Input_Voltage", 144));
  name2addr.insert(std::pair<std::string, int>("Operating_Mode", 11));
  name2addr.insert(std::pair<std::string, int>("Profile_Velocity", 112));
  name2addr.insert(std::pair<std::string, int>("Profile_Acceleration", 108));

  name2length.insert(std::pair<std::string, int>("Return_Delay_Time", 1));
  name2length.insert(std::pair<std::string, int>("Max_Position_Limit", 4));
  name2length.insert(std::pair<std::string, int>("Min_Position_Limit", 4));
  name2length.insert(std::pair<std::string, int>("Torque_Enable", 1));
  name2length.insert(std::pair<std::string, int>("Hardware_Error_Status", 1));
  name2length.insert(std::pair<std::string, int>("Goal_Current", 2));
  name2length.insert(std::pair<std::string, int>("Goal_Position", 4));
  name2length.insert(std::pair<std::string, int>("Present_Current", 2));
  name2length.insert(std::pair<std::string, int>("Present_Velocity", 4));
  name2length.insert(std::pair<std::string, int>("Present_Position", 4));
  name2length.insert(std::pair<std::string, int>("Present_Input_Voltage", 2));
  name2length.insert(std::pair<std::string, int>("Operating_Mode", 1));
  name2length.insert(std::pair<std::string, int>("Profile_Velocity", 4));
  name2length.insert(std::pair<std::string, int>("Profile_Acceleration", 4));

  //2.set port handle
  bool result = false;
  result = setPortHandler(device_name, log);
  if (result == false) return false;
  //3.set communicate baudrate
  result = setBaudrate(baud_rate, log);
  if (result == false) return false;
  //4.set communicate protocol
  result = setPacketHandler(2.0f, log);
  if (result == false) return false;

  return result;
}

/**  
*   @brief      scan api
    Parameters:
*   @param      get_id                [out]a pointer to save the motor id    
*   @param      get_the_number_of_id  [out]a pointer to save the number of motor
*   @param      range                 [in]set the scan scale, max id
*   @param      log                   [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::scan(uint8_t *get_id, uint8_t *get_the_number_of_id, uint8_t range, const char **log)
{
  return scan(get_id, get_the_number_of_id, 0, range, log);
}

/**  
*   @brief      scan api
    Parameters:
*   @param      get_id                [out]a pointer to save the motor id    
*   @param      get_the_number_of_id  [out]a pointer to save the number of motor
*   @param      start_num             [in]set the start id of scan
*   @param      end_num               [in]set the end id of scan
*   @param      log                   [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::scan(uint8_t *get_id, uint8_t *get_the_number_of_id, uint8_t start_num, uint8_t end_num, const char **log)
{
  //1.init variables
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;
  uint8_t id = 0;
  uint8_t id_cnt = 0;
  uint16_t model_number = 0;
  uint8_t get_end_num = end_num;

  //2.judge the end num whether out the id range
  if (get_end_num > 253) 
  {
    get_end_num = 253;
  }

  //3.scan protocol 1.0 motor
  result = setPacketHandler(1.0f, log);
  if (result == false) 
  {
    return false;
  }
  //3.1 according to the start and end id to scan motor
  for (id = start_num; id <= get_end_num; id++)
  {
    sdk_error.dxl_comm_result = packetHandler_->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      if (log != NULL)
      {
        *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
      }
    }
    else
    {
      //3.2 save the current motor id if ping pass
      if (sdk_error.dxl_error != 0)
      {
        if (log != NULL) 
        {
          *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
        }
      }
      get_id[id_cnt++] = id;
      //3.3 save the model of motor 
      id_model.insert(std::pair<int, int>(id, model_number));
    }    
  }
  //3.4 record the number of motor
  if (id_cnt > 0)
  {
    *get_the_number_of_id = id_cnt;
    return result;
  }

  //4.scan protocol 2.0 motor
  result = setPacketHandler(2.0f, log);
  if (result == false) return false;
  //4.1 according to the start and end id to scan motor
  for (id = start_num; id <= get_end_num; id++)
  {
    sdk_error.dxl_comm_result = packetHandler_->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
    // printf("result : %d ,    model_number : %d\n", dxl_comm_result, model_number);
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    }
    else
    {
      //4.2 save the current motor id if ping pass
      if (sdk_error.dxl_error != 0)
      {
        if (log != NULL) 
        {
          *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
        }
      }
      dxl_id[id_cnt] = id;
      get_id[id_cnt++] = id;
      //4.3 save the model of motor 
      id_model.insert(std::pair<int, int>(id, model_number));
    }   
  }
  //4.4 record the number of motor
  if (id_cnt > 0)
  {
    *get_the_number_of_id = id_cnt;
    dxl_cnt = id_cnt;
    return result;
  }
  return result;
}

/**  
*   @brief      reboot special id motor
    Parameters:
*   @param      id        [in]the motor id    
*   @param      log       [out]saving log
*   @return     true:reboot successful; false:failed    
    */
bool DynamixelClass::reboot(uint8_t id, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  if (getProtocolVersion() == 1.0)
  {
    if (log != NULL) *log = "[DynamixelClass] reboot functions is not available with the Dynamixel Protocol 1.0.";
    return false;
  }
  else
  {
    sdk_error.dxl_comm_result = packetHandler_->reboot(portHandler_, id, &sdk_error.dxl_error);
    
    #if defined(__OPENCR__) || defined(__OPENCM904__)
      delay(1000);
    #elif defined(_WIN32)
      std::this_thread::sleep_for(std::chrono::microseconds(1000*1000));
    #else
      usleep(1000*1000);
    #endif

    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      if (log != NULL) *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else
    {
      if (sdk_error.dxl_error != 0) {
        if (log != NULL) *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
      }
      else {
        if (log != NULL) *log = "[DynamixelClass] Succeeded to reboot!";
      }
      return true;
    }
  }

  return false;
}


/**  
*   @brief      according to id_model to judge model name
    Parameters:
*   @param      id        [in]the motor id    
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
const char * DynamixelClass::getModelName(uint8_t id, const char **log)
{
  //1.judge motor model,output correspond to model name
  if(id_model[id] == 311)
  {
    return "MX-64-2";
  }else if(id_model[id] == 30)
  {
    return "MX-28-2";
  }else if(id_model[id] == 321)
  {
    return "MX-106-2";
  }else if(id_model[id] == 1000)
  {
    printf("%d\n",id_model[id]);
    return "don't know";
  }else
  {
    printf("%d\n",id_model[id]);
    return "no model";
  }
}

/**  
*   @brief      torque use api
    Parameters:
*   @param      id        [in]the motor id    
*   @param      onoff     [in]enable or disable 
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::torque(uint8_t id, int32_t onoff, const char **log)
{
  bool result = false;
  //1.write byte to register
  result = itemWrite(id, "Torque_Enable", (int32_t)onoff, log);
  //2.add log 
  if (result == false)
  {
    if (log != NULL)
    { 
      *log = "[DynamixelClass] Failed to change torque status!";
    }
    return false;
  }
  if (log != NULL) 
  {
    *log = "[DynamixelClass] Succeeded to change torque status!";
  }
  return result;
}

/**  
*   @brief      torque enable
    Parameters:
*   @param      id        [in]the motor id    
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::torqueOn(uint8_t id, const char **log)
{
  bool result = false;
  result = torque(id, 1, log);
  return result;
}

/**  
*   @brief      torque disable
    Parameters:
*   @param      id        [in]the motor id    
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::torqueOff(uint8_t id, const char **log)
{
  bool result = false;
  result = torque(id, 0, log);
  return result;
}

/**  
*   @brief      set motor operate mode enable
    Parameters:
*   @param      id        [in]the motor id    
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::setOperatingMode(uint8_t id, const uint8_t model)
{
  //1.according to model choose suitable config api
  switch(model)
  {
    case CURRENT_CONTROL_MODE:
      itemWrite(id, "Operating_Mode", CURRENT_CONTROL_MODE);
      break;
    case VELOCITY_CONTROL_MODE:
      itemWrite(id, "Operating_Mode", VELOCITY_CONTROL_MODE);
      break;
    case POSITION_CONTROL_MODE:
      itemWrite(id, "Operating_Mode", POSITION_CONTROL_MODE);
      break;
    case EXTENDED_POSITION_CONTROL_MODE:
      itemWrite(id, "Operating_Mode", EXTENDED_POSITION_CONTROL_MODE);
      break;
    case CURRENT_BASED_POSITION_CONTROL_MODE:
      itemWrite(id, "Operating_Mode", CURRENT_BASED_POSITION_CONTROL_MODE);
      break;
    case PWM_CONTROL_MODE:
      itemWrite(id, "Operating_Mode", PWM_CONTROL_MODE);
      break;
    case TORQUE_CONTROL_MODE:
      itemWrite(id, "Operating_Mode", TORQUE_CONTROL_MODE);
      break;
    case MULTI_TURN_MODE:
      itemWrite(id, "Operating_Mode", MULTI_TURN_MODE);
      break;
    default: 
      printf("no model\n");
      return false;
  }
  return true;
}

/**  
*   @brief      set motor position operate mode
    Parameters:
*   @param      id        [in]the motor id    
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::setPositionControlMode(uint8_t id, const char **log)
{
  //1.init variables
  bool result = false;
  //2.set operate mode
  result = setOperatingMode(id, POSITION_CONTROL_MODE);
  //3.add log information
  if(result == false)
  {
    if (log != NULL) 
    {
      *log = "[DynamixelClass] Failed to set Position Control Mode!";
    }
    return false;
  }
  if(log != NULL) 
  {
    *log = "[DynamixelClass] Succeeded to set Position Control Mode!";
  }
  return result;
}

/**  
*   @brief      set motor current based position operate mode
    Parameters:
*   @param      id        [in]the motor id    
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::setCurrentBasedPositionControlMode(uint8_t id, const char **log)
{
  //1.init variables
  bool result = false;
  //2.set operate mode
  result = setOperatingMode(id, CURRENT_BASED_POSITION_CONTROL_MODE);
  //3.add log information
  if (result == false)
  {
    if (log != NULL) 
    {
      *log = "[DynamixelClass] Failed to set Current Based Position Control Mode!";
    }
    return false;
  }
  if (log != NULL) 
  {
    *log = "[DynamixelClass] Succeeded to set Current Based Position Control Mode!";
  }
  return result;
}

/**  
*   @brief      add synchronization write handle
    Parameters:
*   @param      id        [in]just for correspond workbench api, no actual meaning    
*   @param      item_name [in]the attribute name that want add to sync write handle
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::addSyncWriteHandler(uint8_t id, const char *item_name, const char **log)
{
  //1.judge the number of sync write handle whether granter than MAX_HANDLER_NUM
  if (sync_write_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    //2.if greater,add log information
    if (log != NULL) *log = "[DynamixelClass] Too many sync write handler are added (MAX = 5)";
    return false;
  }
  //3.if not, get the address and length of attribute from name2addr and name2length
  auto iter = name2addr.find(item_name);
  //4.if get the attribute, add sync write handle
  if(iter != name2addr.end())
  {
    syncWriteHandler_[sync_write_handler_cnt_].control_item = NULL;
    syncWriteHandler_[sync_write_handler_cnt_].groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_,
                                                                                          packetHandler_,
                                                                                          iter->second,
                                                                                          name2length[item_name]);
    sync_write_handler_cnt_++;
    if (log != NULL) 
    {
      *log = "[DynamixelClass] Succeeded to add sync write handler";
    }
    return true;  
  }else
  {   
      //5.if not, output prompt message
      std::cout <<  "can't add '" << item_name << "' to syncWriteHandle;" << std::endl;
      return false;
  }       
}

/**  
*   @brief      add synchronization read handle
    Parameters:
*   @param      id        [in]just for correspond workbench api, no actual meaning    
*   @param      item_name [in]the attribute name that want add to sync write handle
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::addSyncReadHandler(uint8_t id, const char *item_name, const char **log)
{
  //1.judge the number of sync write handle whether granter than MAX_HANDLER_NUM
  if (sync_read_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    if (log != NULL) 
    {
      //2.if greater,add log information
      *log = "[DynamixelClass] Too many sync read handler are added (MAX = 5)";
    }
    return false;
  }
  //3.if not, get the address and length of attribute from name2addr and name2length
  auto iter = name2addr.find(item_name);
  //4.if get the attribute, add sync write handle and address and length to syncReadHandler_
  if(iter != name2addr.end())
  {
    syncReadHandler_[sync_read_handler_cnt_].address = (uint16_t)iter->second;
    syncReadHandler_[sync_read_handler_cnt_].data_length = name2length[item_name];
    syncReadHandler_[sync_read_handler_cnt_].groupSyncRead = new dynamixel::GroupSyncRead(portHandler_,
                                                                                          packetHandler_,
                                                                                          iter->second,
                                                                                          name2length[item_name]);                                                                                 
    sync_read_handler_cnt_++;
    if (log != NULL) 
    {
      *log = "[DynamixelClass] Succeeded to add sync read handler";
    }
    return true;  
  }else
  {
    //5.if not, output prompt message
    std::cout <<  "can't add '" << item_name << "' to syncReadHandle;" << std::endl;
    return false;
  }       
}

/**  
*   @brief      transfer the data to the four one byte param 
    Parameters:
*   @param      data    [in]the data waiting for transfer    
*   @param      param   [in]the param saving data
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
void DynamixelClass::getParam(int32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}

/**  
*   @brief      synchronization write data 
    Parameters:
*   @param      index     [in]the serial number of sync write handle, indicate different attribute waiting for sync write
*   @param      data      [in]the data waiting for sync send
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::syncWrite(uint8_t index, int32_t *data, const char **log)
{
  //1.init variables
  ErrorFromSDK sdk_error = {0, false, false, 0};
  uint8_t count = 0;
  uint8_t parameter[4] = {0, 0, 0, 0};
  static uint8_t temp_index = 0;


  //2.make ready data waiting for send
  for (int i = 0; i < dxl_cnt; i++)
  {
    //3.make ready data
    temp_index = dxl_id[count] - 1;
    assert(MAX_ID_NUM >= temp_index);
    getParam(data[temp_index], parameter);
    // getParam(data[count], parameter);
    //4.add data and motor id to sync write handle 
    sdk_error.dxl_addparam_result = syncWriteHandler_[index].groupSyncWrite->addParam(dxl_id[count], (uint8_t *)&parameter);
    if (sdk_error.dxl_addparam_result != true)
    {
      if (log != NULL) 
      {
        printf("groupSyncWrite addparam failed, motor id:%d\n", dxl_id[count]);
        *log = "groupSyncWrite addparam failed";
      }
      return false;
    }
    else
    {
      count++;
    }
  }
  //5.send the sync write data
  sdk_error.dxl_comm_result = syncWriteHandler_[index].groupSyncWrite->txPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) 
    {
      *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    }
    return false;
  }
  //6.clear the sync write handle
  syncWriteHandler_[index].groupSyncWrite->clearParam();
  if (log != NULL) 
  {
    *log = "[DynamixelClass] Succeeded to sync write!";
  }
  return true;
}

/**  
*   @brief      synchronization read data 
    Parameters:
*   @param      index     [in]the serial number of sync read handle, indicate different attribute waiting for sync write
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::syncRead(uint8_t index, const char **log)
{
  //1.init variables
  ErrorFromSDK sdk_error = {0, false, false, 0};

  //2.clear previous sync read handle
  syncReadHandler_[index].groupSyncRead->clearParam();
  for (int i = 0; i < dxl_cnt; i++)
  {
    //3.add motor id to sync read handle
    assert(MAX_ID_NUM >= dxl_id[i]);
    sdk_error.dxl_addparam_result = syncReadHandler_[index].groupSyncRead->addParam(dxl_id[i]);
    if (sdk_error.dxl_addparam_result != true)
    {
      if (log != NULL) 
      {
        *log = "groupSyncWrite addparam failed";
      }
      return false;
    }
  }
  //4.send sync read packet
 sdk_error.dxl_comm_result = syncReadHandler_[index].groupSyncRead->txRxPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    if (log != NULL) 
    {
      *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    }
    return false;
  }
  if (log != NULL) 
  {
    *log = "[DynamixelClass] Succeeded to sync read!";
  }
  return true;
}  

/**  
*   @brief      get synchronization read data 
    Parameters:
*   @param      index     [in]the serial number of sync read handle, indicate different attribute waiting for sync write
*   @param      data      [in]save the sync read data
*   @param      log       [out]saving log
*   @return     true:write successful; false:failed    
    */
bool DynamixelClass::getSyncReadData(uint8_t index, int32_t *data, const char **log)
{
  //1.init variables
  ErrorFromSDK sdk_error = {0, false, false, 0};
  static uint8_t temp_index = 0;

  for (int i = 0; i < dxl_cnt; i++)
  {
    //2.check the data is available?
    temp_index = dxl_id[i]-1;
    assert(MAX_ID_NUM >= temp_index);
    assert(MAX_ID_NUM >= dxl_id[i]);
    sdk_error.dxl_getdata_result = syncReadHandler_[index].groupSyncRead->isAvailable(dxl_id[i], 
                                                      syncReadHandler_[index].address, 
                                                      syncReadHandler_[index].data_length);
    if(sdk_error.dxl_getdata_result != true)
    {
      if (log != NULL) 
      {
        *log = "groupSyncRead getdata failed";
      }
      return false;
    }
    else
    {
      //3.save the data to the pointer
      data[temp_index] = syncReadHandler_[index].groupSyncRead->getData(dxl_id[i], 
                                                      syncReadHandler_[index].address, 
                                                      syncReadHandler_[index].data_length);
    }
  }
  //4.save log
  if (log != NULL) 
  {
    *log = "[DynamixelClass] Succeeded to get sync read data!";
  }
  return true;
}

/**  
*   @brief      get the version of protocol
    Parameters:
*   @return     the version of protocol  
    */
float DynamixelClass::getProtocolVersion(void)
{
  return packetHandler_->getProtocolVersion();
}

/**  
*   @brief      convert the current to value,set motor current
    Parameters:
*   @param      id      [in]motor id
*   @param      current [in]the current waiting for convert
*   @return     convert value
    */
int16_t DynamixelClass::convertCurrent2Value(uint8_t id, float current)
{
  //1.init variable
  float CURRENT_UNIT = 2.69f; //Unit : mA, Ref : http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102
  //judge the protocol
  if (getProtocolVersion() == 1.0f)
  {
    return (current / CURRENT_UNIT);
  }
  else if (getProtocolVersion() == 2.0f)
  {
    if(strncmp(getModelName(id), "PRO-L", strlen("PRO-L")) == 0 ||
      strncmp(getModelName(id), "PRO-M", strlen("PRO-M")) == 0 ||
      strncmp(getModelName(id), "PRO-H", strlen("PRO-H")) == 0)
    {
      CURRENT_UNIT = 16.11328f;
      return (current / CURRENT_UNIT);
    }
    else if (strncmp(getModelName(id), "PRO-PLUS", strlen("PRO-PLUS")) == 0)
    {
      CURRENT_UNIT = 1.0f;
      return (current / CURRENT_UNIT);
    }
    else
    {
      return (current / CURRENT_UNIT);
    }
  }
  return (current / CURRENT_UNIT);
}

/**  
*   @brief      convert the value to radian,motor feedback
    Parameters:
*   @param      id      [in]motor id
*   @param      value   [in]the value waiting for convert
*   @return     convert radian
    */
float DynamixelClass::convertValue2Radian(uint8_t id, int32_t value)
{
  float radian = 0.0;
  int Model_max_position=4095;
  double modelCenter=Model_max_position/2.;
  radian=(((double)value - modelCenter)*3.14159/modelCenter);
  return radian;
}

/**  
*   @brief      convert the value to velocity,motor feedback
    Parameters:
*   @param      id      [in]motor id
*   @param      value   [in]the value waiting for convert
*   @return     convert velocity
    */
float DynamixelClass::convertValue2Velocity(uint8_t id, int32_t value)
{
  //1.init variables
  float velocity = 0;
  const float RPM2RADPERSEC = 0.104719755f;
  float rpm = 1.0;
  //2.judge protocol
  if (getProtocolVersion() == 1.0f)
  {
    if (strncmp(getModelName(id), "AX", strlen("AX")) == 0 ||
        strncmp(getModelName(id), "RX", strlen("RX")) == 0 ||
        strncmp(getModelName(id), "EX", strlen("EX")) == 0 ||
        strncmp(getModelName(id), "MX", strlen("MX")) == 0)
    {
      if (value == 1023 || value == 0) velocity = 0.0f;
      else if (value > 0 && value < 1023) velocity = value * rpm * RPM2RADPERSEC;
      else if (value > 1023 && value < 2048) velocity = (value - 1023) * rpm * RPM2RADPERSEC  * (-1.0f);

      return velocity;
    }
  }
  else if (getProtocolVersion() == 2.0f)
  {
    if (strcmp(getModelName(id), "XL-320") == 0)
    {
      if (value == 1023 || value == 0) velocity = 0.0f;
      else if (value > 0 && value < 1023) velocity = value * rpm * RPM2RADPERSEC;
      else if (value > 1023 && value < 2048) velocity = (value - 1023) * rpm * RPM2RADPERSEC * (-1.0f);
    }
    else
    {
      velocity = value * (rpm * RPM2RADPERSEC);
    }
    return velocity;
  }
  return 0.0f;
}

/**  
*   @brief      convert the value to current,motor feedback
    Parameters:
*   @param      value   [in]the value waiting for convert
*   @return     convert current
    */
float DynamixelClass::convertValue2Current(int16_t value)
{
  float current = 0;
  const float CURRENT_UNIT = 2.69f; //Unit : mA, Ref : http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102
  current = (int16_t)value * CURRENT_UNIT;
  return current;
}


#endif