/*
 * mIDriver.cpp
 * This is 2g ambot robot low driver mi motor driver program
 * Created on: march 1, 2024
 *      Author: chen chen
 */
#include "ambotDriver.hpp"
#include <sys/time.h>
#include "assert.h"

/**  
*   @brief      the construct function of MCU class
    Parameters:
*   @param      deviceName    	[in]the device name of mcu
*   @param      baudRate 		[in]communicate baud rate
*   @return     none
    */
AmbotDriver::AmbotDriver(AmbotDeviceTPDF& inputParams) : ambotFeatures(inputParams)
{
    // 1.open device file
    ambotMcuFd = open(ambotFeatures.motorDevName.c_str(), O_RDWR | O_NOCTTY );
    if (ambotMcuFd == -1) {
        perror("Error opening motor serial port");
        exit(0);
    }
    // 2.init serial config file
    struct termios SerialPortSettings;
    struct serial_struct ser_info;
    // 3.1 get serial current feature and save to SerialPortSettings
    tcgetattr(ambotMcuFd, &SerialPortSettings);
    // 3.2 according baud to set the serial baudrate
    if (ambotFeatures.motorDevBaud == 115200)
    {
        std::cout << COUT_BLUE << "motor set baud:" << 115200 << COUT_RESET << std::endl;
        cfsetispeed(&SerialPortSettings, B115200);
        cfsetospeed(&SerialPortSettings, B115200);
    }else if (ambotFeatures.motorDevBaud == 921600)
    {
        std::cout << COUT_BLUE << "motor set baud:" << 921600 << COUT_RESET << std::endl;
        cfsetispeed(&SerialPortSettings, B921600);
        cfsetospeed(&SerialPortSettings, B921600);
    }else if (ambotFeatures.motorDevBaud == 1000000)
    {
        std::cout << COUT_BLUE << "motor set baud:" << 1000000 << COUT_RESET << std::endl;
        cfsetispeed(&SerialPortSettings, B1000000);
        cfsetospeed(&SerialPortSettings, B1000000);
    }else if (ambotFeatures.motorDevBaud == 1500000)
    {
        std::cout << COUT_BLUE << "motor set baud:" << 1500000 << COUT_RESET << std::endl;
        cfsetispeed(&SerialPortSettings, B1500000);
        cfsetospeed(&SerialPortSettings, B1500000);
    }else if (ambotFeatures.motorDevBaud == 2000000)
    {
        std::cout << COUT_BLUE << "motor set baud:" << 2000000 << COUT_RESET << std::endl;
        cfsetispeed(&SerialPortSettings, B2000000);
        cfsetospeed(&SerialPortSettings, B2000000);
    }else if (ambotFeatures.motorDevBaud == 2500000)
    {
        std::cout << COUT_BLUE << "motor set baud:" << 2500000 << COUT_RESET << std::endl;
        cfsetispeed(&SerialPortSettings, B2500000);
        cfsetospeed(&SerialPortSettings, B2500000);
    }
    // 3.3 set uart communicate features
    SerialPortSettings.c_cflag &= ~PARENB;      // Disable parity
    SerialPortSettings.c_cflag &= ~CSTOPB;      // 1 stop bit
    SerialPortSettings.c_cflag |= CS8;          // 8 bits per byte
    SerialPortSettings.c_cflag &= ~CRTSCTS;     // Disable RTS/CTS hardware flow control
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = SerialPortSettings
    SerialPortSettings.c_lflag &= ~ICANON;      // Disable canonical mode
    SerialPortSettings.c_lflag &= ~ECHO;        // Disable echo
    SerialPortSettings.c_lflag &= ~ECHOE;       // Disable erasure
    SerialPortSettings.c_lflag &= ~ECHONL;      // Disable new-line echo
    SerialPortSettings.c_lflag &= ~ISIG;        // Disable interpretation of INTR, QUIT and SUSP
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    SerialPortSettings.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    SerialPortSettings.c_oflag &= ~OPOST;       // Prevent special interpretation of output bytes (e.g. newline chars)
    SerialPortSettings.c_oflag &= ~ONLCR;       // Prevent conversion of newline to carriage return/line feed
    SerialPortSettings.c_cc[VTIME] = 0;         // timeout set, unit 1/10s, if set zero, will return right now 
    SerialPortSettings.c_cc[VMIN] = 0;          // wait for enough data to read, if set zero, data will read right now
    // 3.4 Enable linux FTDI low latency mode
    ioctl(ambotMcuFd, TIOCGSERIAL, &ser_info);
    ser_info.flags |= ASYNC_LOW_LATENCY;
    ioctl(ambotMcuFd, TIOCSSERIAL, &ser_info);
    // 3.5 Set the new attributes to the termios structure
	if((tcsetattr(ambotMcuFd, TCSANOW, &SerialPortSettings)) != 0) 
    {
        std::cout << "ERROR in setting serial port attributes! for port " << ambotFeatures.motorDevName.c_str() << std::endl;
        sleep(1);
    }
	else
		std::cout << "Port " << ambotFeatures.motorDevName.c_str() << " open successfully!" << endl;
    
    // 4. set sensor serial feature
    sensorFd = open(ambotFeatures.sensorDevName.c_str(), O_RDWR | O_NOCTTY );
    if (sensorFd == -1) {
        perror("Error opening sensor serial port");
        printf("imu can't find!!!\n");
    }else{
        printf("imu find!!!\n");
        tcgetattr(sensorFd, &SerialPortSettings);
        // printf("sensor baudrate:%d\n", ambotFeatures.sensorDevBaud);
        if (ambotFeatures.sensorDevBaud == 115200)
        {
            std::cout << COUT_BLUE << "sensor set baud:" << 115200 << COUT_RESET << std::endl;
            cfsetispeed(&SerialPortSettings, B115200);
            cfsetospeed(&SerialPortSettings, B115200);
        }else if (ambotFeatures.sensorDevBaud == 921600)
        {
            std::cout << COUT_BLUE << "sensor set baud:" << 921600 << COUT_RESET << std::endl;
            cfsetispeed(&SerialPortSettings, B921600);
            cfsetospeed(&SerialPortSettings, B921600);
        }else if (ambotFeatures.sensorDevBaud == 1000000)
        {
            std::cout << COUT_BLUE << "sensor set baud:" << 1000000 << COUT_RESET << std::endl;
            cfsetispeed(&SerialPortSettings, B1000000);
            cfsetospeed(&SerialPortSettings, B1000000);
        }else if (ambotFeatures.sensorDevBaud == 1500000)
        {
            std::cout << COUT_BLUE << "sensor set baud:" << 1500000 << COUT_RESET << std::endl;
            cfsetispeed(&SerialPortSettings, B1500000);
            cfsetospeed(&SerialPortSettings, B1500000);
        }else if (ambotFeatures.sensorDevBaud == 2000000)
        {
            std::cout << COUT_BLUE << "sensor set baud:" << 2000000 << COUT_RESET << std::endl;
            cfsetispeed(&SerialPortSettings, B2000000);
            cfsetospeed(&SerialPortSettings, B2000000);
        }else if (ambotFeatures.sensorDevBaud == 2500000)
        {
            std::cout << COUT_BLUE << "sensor set baud:" << 2500000 << COUT_RESET << std::endl;
            cfsetispeed(&SerialPortSettings, B2500000);
            cfsetospeed(&SerialPortSettings, B2500000);
        }
        SerialPortSettings.c_cflag &= ~PARENB;      // Disable parity
        SerialPortSettings.c_cflag &= ~CSTOPB;      // 1 stop bit
        SerialPortSettings.c_cflag |= CS8;          // 8 bits per byte
        SerialPortSettings.c_cflag &= ~CRTSCTS;     // Disable RTS/CTS hardware flow control
        SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = SerialPortSettings
        SerialPortSettings.c_lflag &= ~ICANON;      // Disable canonical mode
        SerialPortSettings.c_lflag &= ~ECHO;        // Disable echo
        SerialPortSettings.c_lflag &= ~ECHOE;       // Disable erasure
        SerialPortSettings.c_lflag &= ~ECHONL;      // Disable new-line echo
        SerialPortSettings.c_lflag &= ~ISIG;        // Disable interpretation of INTR, QUIT and SUSP
        SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        SerialPortSettings.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
        SerialPortSettings.c_oflag &= ~OPOST;       // Prevent special interpretation of output bytes (e.g. newline chars)
        SerialPortSettings.c_oflag &= ~ONLCR;       // Prevent conversion of newline to carriage return/line feed
        SerialPortSettings.c_cc[VTIME] = 0;         // timeout set, unit 1/10s, if set zero, will return right now 
        SerialPortSettings.c_cc[VMIN] = 0;          // wait for enough data to read, if set zero, data will read right now
        ioctl(sensorFd, TIOCGSERIAL, &ser_info);
        ser_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(sensorFd, TIOCSSERIAL, &ser_info);
        if((tcsetattr(sensorFd, TCSANOW, &SerialPortSettings)) != 0) 
        {
            std::cout << "ERROR in setting sensor serial port attributes! for port " << ambotFeatures.sensorDevName << std::endl;
            sleep(1);
        }
        else
            std::cout << "Port " << ambotFeatures.sensorDevName << " open successfully!" << endl;
    }

    // init variables
    motorLimit = {4*MY_PI, -4*MY_PI, 30, -30, 12, -12, 500., 0, 5., 0};
    ambotState.motorState.resize(ambotFeatures.motorNum);
    for (int i = 0; i < ambotFeatures.motorNum; i++)
    {
        ambotState.motorState[i].pos = 0.0;
        ambotState.motorState[i].cur = 0.0;
        ambotState.motorState[i].vel = 0.0;
        ambotState.motorState[i].vol = 0.0;
    }
    
    ambotState.motor_num = ambotFeatures.motorNum;
    motorNum = 0;
}

/**  
*   @brief      the deconstruct function of MCU class
    Parameters:
*   @param      
*   @return     none
    */
AmbotDriver::~AmbotDriver()
{
    setAllMotorOneFeature(ControlCommandCode.DISABLE_MOTOR, (uint32_t)0);
    printf("program end\n");
}

/**  
*   @brief      new thread to update motor feedback data
    Parameters:
*   @param      arg   	[in]this is a pointer to deliver parameter to thread
*   @return     none  
    */
void* AmbotDriver::newReadMotorThread(void* arg)
{
    AmbotDriver* ptr = (AmbotDriver*) arg;
    ptr->getAllMotorStateFromMCU();
    pthread_exit(0);
}

/**  
*   @brief      new thread to update imu and force sensor data
    Parameters:
*   @param      arg   	[in]this is a pointer to deliver parameter to thread
*   @return     none  
    */
void* AmbotDriver::newReadSensorThread(void* arg)
{
    AmbotDriver* ptr = (AmbotDriver*) arg;
    ptr->getSensorDataFromMCU();
    pthread_exit(0);
}

// /**  
// *   @brief      create a data to save log 
//     Parameters:
// *   @param      data    [in]a map data save all need to log
// *   @return     none  
//     */
// void AmbotDriver::createLogData(std::vector<pair<string, float>>& data)
// {
//     data.clear();
//     // for (int i = 0; i < motorNum; i++)
//     // {
//     //     data.push_back(pair<string, float>("cmd_" + to_string(i), position.at(i)));
//     // }
//     // for (int i = 0; i < motorNum; i++)
//     // {
//     //     data.push_back(pair<string, float>("feedback_" + to_string(i), positionFeedback.at(i)));
//     // }
// }

/**  
*   @brief      initial function for MCU class
    Parameters:
*   @param      void
*   @return     none
    */
bool AmbotDriver::initial(void)
{
    threadStop = false;
    printf("ready to init\n\n");
    if(getAllMotorInformation())
    {
        std::cout << COUT_GREEN << "getID OK!!!\n" <<  COUT_RESET << std::endl;
        motorID.resize(motorNum);

        // set zero in real robot node, need to delete it
        // setAllMotorOneFeature(ControlCommandCode.SET_ZERO, (uint32_t)0);
        setAllMotorOneFeature(ControlCommandCode.ENABLE_MOTOR, (uint32_t)0);
        setAllMotorOneFeature(ControlCommandCode.SELECT_MODE, ambotFeatures.motorControlMode);
        setAllMotorOneFeature(ControlCommandCode.SET_MAX_ANGLE, ambotFeatures.motorMaxAngle);
        setAllMotorOneFeature(ControlCommandCode.SET_MIN_ANGLE, ambotFeatures.motorMinAngle);
        if (ambotFeatures.motorControlMode == 1)
            setAllMotorOneFeature(ControlCommandCode.SET_RUNNING_VELOCITY, ambotFeatures.motorVelocity);
        createReceiveThread();
        return true;
    }
    else
    {
        std::cout << COUT_RED << "getID failed!!!\n" <<  COUT_RESET << std::endl;
        return false;
    }
}

/**  
*   @brief      create a receive data analysis thread
    Parameters:
*   @param      void
*   @return     none
    */
void AmbotDriver::createReceiveThread(void)
{
    if(pthread_create(&motorTid, NULL, newReadMotorThread, (void *)this) != 0)
        perror("Create read mcu data thread fail!\n");
    if (sensorFd == -1)
        printf("No sensor serial port!!!\n");
    else
        if(pthread_create(&sensorTid, NULL, newReadSensorThread, (void *)this) != 0)
            perror("Create sensor data read thread fail!\n");
}

/**  
*   @brief      analysis receive uart data and update motor state
    Parameters:
*   @param      buffer      [in]a pointer for receive uart data,wait analysis 
*   @param      jointState  [out]a reference of all motor feedback state, wait update 
*   @return     none
    */
void AmbotDriver::allMotorStateAnalysis(uint8_t* buffer, ambot_msgs::JointState& jointState)
{
    uint16_t temp = 0;
    temp = (*(buffer + 0) << 8) | (*(buffer + 1)) ; 
    jointState.pos = uint16ToFloat(temp, motorLimit.minPosition, motorLimit.maxPosition);   //range:-4pi~4pi
    temp = (*(buffer + 2) << 8) | (*(buffer + 3)); 
    jointState.vel = uint16ToFloat(temp, motorLimit.minVelocity, motorLimit.maxVelocity);   //range:-30~30 rad/s
    temp = (*(buffer + 4) << 8) | (*(buffer + 5)); 
    jointState.cur = uint16ToFloat(temp, motorLimit.minCurrent, motorLimit.maxCurrent);     //range:-12~12 N*m
}

/**  
*   @brief      send the tty data to mcu
    Parameters:
*   @param      void
*   @return     the num of send data
    */
ssize_t AmbotDriver::txPacket(void)
{
    return  write(ambotMcuFd, writeBuffer, dataLength);
}


/**  
*   @brief      create command data frame for control command
    Parameters:
*   @param      commandNum    	[in]the command code
*   @param      data 			[in]the data(unsigned int) wait to send 
*   @return     none
    */
template<typename T>
void AmbotDriver::createCommandFrame(const uint8_t functionCode, const uint8_t commandNum, const std::vector<T>& data)
{
    memset(writeBuffer, 0, WRITE_BUFFER_SIZE);
    writeBuffer[ControlIndex.FRAME_HEAD] = FRAME_HEAD_DATA;
    writeBuffer[ControlIndex.FUNCTION_CODE] = functionCode;
    writeBuffer[ControlIndex.COMMAND_CODE] = commandNum;
    writeBuffer[ControlIndex.MOTOR_NUM] = motorNum;
    dataLength = ControlIndex.DATA_LENGTH;
    for (int i = 0; i < motorNum; i++)
    {
        writeBuffer[ControlIndex.MOTOR_ID + ControlIndex.MOTOR_DATA_SIZE*i] = motorID[i];
        //keep the position of data corresponding to the motor id
        otherTransferToByte(data[i], &writeBuffer[ControlIndex.MOTOR_DATA + ControlIndex.MOTOR_DATA_SIZE*i]);
        dataLength += ControlIndex.MOTOR_DATA_SIZE;
    }
    generateCRC(&writeBuffer[ControlIndex.FUNCTION_CODE], dataLength - 1);
    dataLength += CRC_DATA_SIZE;
}


/**  
*   @brief      wait the response data and analyses data
    Parameters:
*   @param      commandNumber		[in]the command code 
*   @return     true:get successful   false:get failed
    */
bool AmbotDriver::waitCommandResponse(const uint8_t commandNumber)
{
    // 1.init all local variables
    struct timeval time_old,time_new;
    uint8_t crcAuthentication[CRC_DATA_SIZE];
    uint8_t tempReadBuffer[READ_BUFFER_SIZE];
    uint8_t currentReadCount, alreadyReadCount, frameHeadIndex;
    bool frameHeadFlag = false;
    frameHeadIndex = 0;
    alreadyReadCount = 0;
    // 2.get current time for read timeout
    gettimeofday(&time_new, NULL);
    time_old = time_new;
    // 3.wait for receive data
    while (((time_new.tv_sec*U_SECOND_COUNT + time_new.tv_usec)-(time_old.tv_sec*U_SECOND_COUNT + time_old.tv_usec)) < DELAY_TIMEOUT )
    {
        usleep(WAIT_RESPONSE_US_DELAY);
        currentReadCount = read(ambotMcuFd, &tempReadBuffer[alreadyReadCount], READ_BUFFER_SIZE - alreadyReadCount);
        alreadyReadCount += currentReadCount;
        // 4.judge current receive data length whether if enough to analysis
        if (alreadyReadCount - frameHeadIndex >= 5)
        {
            // 5.locate frame head index
            for (int i = frameHeadIndex; i < alreadyReadCount; i++)
            {
                if (tempReadBuffer[i + ControlIndex.FRAME_HEAD] == FRAME_HEAD_DATA && 
                    tempReadBuffer[i + ControlIndex.FUNCTION_CODE] == FunctionCode.CONTROL_RESPOND)
                {
                    frameHeadIndex = i;
                    frameHeadFlag = true;
                    break;
                }
                frameHeadFlag = false;
            }
            // 6.if frame head locate successful, ready to analysis
            if (frameHeadFlag)
            {
                // printf("command code:%x\n", commandNumber);
                memcpy(readBuffer, &tempReadBuffer[frameHeadIndex + ControlIndex.FUNCTION_CODE], ResponseIndex.COMMAND_CODE + 1);
                for (int i = 0; i < 2; i++)
                {
                    printf("%x ", readBuffer[i]);
                }
                printf("\n");
                memcpy(crcAuthentication, &tempReadBuffer[frameHeadIndex + ResponseIndex.CRC_AUTHENTICATION + 1], CRC_DATA_SIZE);
                if (readBuffer[ResponseIndex.COMMAND_CODE] == commandNumber)
                {
                    generateCRC(readBuffer, ResponseIndex.CRC_AUTHENTICATION);
                    if (!memcmp(&readBuffer[ResponseIndex.CRC_AUTHENTICATION], crcAuthentication, CRC_DATA_SIZE))
                        return true;
                }
                frameHeadIndex++;
            }
        }
        gettimeofday(&time_new, NULL);
    }
    printf("timeout\n");
    return false;
}

/**  
*   @brief      wait the request response data and analyses data
    Parameters:
*   @param      commandNumber		[in]the command code 
*   @return     true:get successful   false:get failed
    */
bool AmbotDriver::waitRequestResponse(const uint8_t commandNumber)
{
    // 1.init all local variables
    struct timeval time_old,time_new;
    uint8_t crcAuthentication[CRC_DATA_SIZE];
    uint8_t tempReadBuffer[READ_BUFFER_SIZE];
    uint8_t resMotorNum, currentReadCount, alreadyReadCount;
    bool frameHeadFlag = false;
    uint8_t frameHeadIndex = 0;
    alreadyReadCount = 0;
    // 2.get current time for read timeout
    gettimeofday(&time_new, NULL);
    time_old = time_new;
    // 3.wait for receive data
    while (((time_new.tv_sec*U_SECOND_COUNT + time_new.tv_usec) - (time_old.tv_sec*U_SECOND_COUNT + time_old.tv_usec)) < DELAY_TIMEOUT )
    {
        usleep(WAIT_RESPONSE_US_DELAY);
        currentReadCount = read(ambotMcuFd, &tempReadBuffer[alreadyReadCount], READ_BUFFER_SIZE - alreadyReadCount);
        alreadyReadCount += currentReadCount;
        // 4.judge current receive data length whether if enough to analysis
        if( alreadyReadCount - frameHeadIndex > 5)
        {
            // 5.locate frame head index
            for (int i = frameHeadIndex; i < alreadyReadCount; i++)
            {
                if (tempReadBuffer[i + ControlIndex.FRAME_HEAD] == FRAME_HEAD_DATA && 
                    tempReadBuffer[i + ControlIndex.FUNCTION_CODE] == FunctionCode.REQUEST_RESPOND)
                {
                    frameHeadIndex = i;
                    frameHeadFlag = true;
                    break;
                }
                frameHeadFlag = false;
            }
            // 6.if frame head locate successful, ready to analysis
            if (frameHeadFlag)
            {
                if (tempReadBuffer[frameHeadIndex + ControlIndex.COMMAND_CODE] == commandNumber)
                {
                    resMotorNum = tempReadBuffer[frameHeadIndex + ControlIndex.MOTOR_NUM];
                    uint8_t dataNum = resMotorNum * ResponseIndex.MOTOR_DATA_SIZE;
                    // do not copy the frame head
                    memcpy(readBuffer, &tempReadBuffer[frameHeadIndex + ControlIndex.FUNCTION_CODE], dataNum + ControlIndex.MOTOR_NUM);
                    memcpy(crcAuthentication, &tempReadBuffer[frameHeadIndex + 4 + dataNum], CRC_DATA_SIZE);
                    generateCRC(readBuffer, dataNum + ResponseIndex.DATA_LENGTH);
                    if (!memcmp(&readBuffer[dataNum + ResponseIndex.DATA_LENGTH], crcAuthentication, CRC_DATA_SIZE))
                    {
                        if(commandNumber == RequestCommandCode.GET_ID && motorNum == 0 && resMotorNum != 0)
                        {
                            motorNum = resMotorNum;
                            motorID.resize(motorNum);
                        }
                        return true;
                    }
                }else
                    frameHeadIndex++;
            }
        }
        gettimeofday(&time_new, NULL);
    }
    return false;
}

/**  
*   @brief      motor set,include: motor mode, max angle. min angle, motor velocity
    Parameters:
*   @param      inputCommandCode    [in]input command code
*   @param      inputData           [in]set data
*   @return     true:get successful   false:get failed
    */
template<typename T> bool AmbotDriver::setAllMotorOneFeature(const uint8_t inputCommandCode, const T inputData)
{
    std::vector<T> setData;
    setData.resize(motorNum);
    for (int i = 0; i < motorNum; i++)
        setData[i] = inputData;
    createCommandFrame(FunctionCode.CONTROL, inputCommandCode, setData);
    if(dataLength != txPacket())
        return false;
    if(!waitCommandResponse(inputCommandCode))
    {
        if (inputCommandCode == ControlCommandCode.SET_ZERO)
            std::cout << COUT_RED << "set motor machine zero Failed!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.ENABLE_MOTOR)
            std::cout << COUT_RED << "enable all motor Failed!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.DISABLE_MOTOR)
            std::cout << COUT_RED << "disable all motor Failed!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.SELECT_MODE)
            std::cout << COUT_RED << "set motor mode Failed!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.SET_MAX_ANGLE)
            std::cout << COUT_RED << "set max limit angle Failed!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.SET_MIN_ANGLE)
            std::cout << COUT_RED << "set min limit angle Failed!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.SET_RUNNING_VELOCITY)
            std::cout << COUT_RED << "set motor velocity Failed!!!" <<  COUT_RESET << std::endl;
        return false;
    }
    else
    {
        if (inputCommandCode == ControlCommandCode.SET_ZERO)
            std::cout << COUT_GREEN << "set motor machine zero OK!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.ENABLE_MOTOR)
            std::cout << COUT_GREEN << "enable all motor OK!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.DISABLE_MOTOR)
            std::cout << COUT_GREEN << "disable all motor OK!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.SELECT_MODE)
            std::cout << COUT_GREEN << "set motor mode OK!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.SET_MAX_ANGLE)
            std::cout << COUT_GREEN << "set max limit angle OK!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.SET_MIN_ANGLE)
            std::cout << COUT_GREEN << "set min limit angle OK!!!" <<  COUT_RESET << std::endl;
        else if (inputCommandCode == ControlCommandCode.SET_RUNNING_VELOCITY)
            std::cout << COUT_GREEN << "set motor velocity OK!!" <<  COUT_RESET << std::endl;
        return true;
    }
}

/**  
*   @brief      motor set,include: motor mode, max angle. min angle, motor velocity
    Parameters:
*   @param      inputCommandCode    [in]input command code
*   @param      inputData           [in]set data
*   @return     true:get successful   false:get failed
    */
template<typename T> bool AmbotDriver::setAllMotorOneFeature(const uint8_t inputCommandCode, const std::vector<T> inputData)
{
    createCommandFrame(FunctionCode.CONTROL, inputCommandCode, inputData);
    if(dataLength != txPacket())
        return false;
    return true;
}

/**  
*   @brief      disable all motor, the interface open to user
    Parameters:
*   @param      none
*   @return     true:get successful   false:get failed
    */
bool AmbotDriver::disableAllMotor(void)
{
    // try three times to stop 
    for (int i = 0; i < 3; i++)
    {
        std::cout << COUT_BLUE << "try " << i+1 << " to stop robot"  << COUT_RESET << std::endl;
        if(setAllMotorOneFeature(ControlCommandCode.DISABLE_MOTOR, (uint32_t)0))
            break;
    }
    // setAllMotorOneFeature(ControlCommandCode.DISABLE_MOTOR, (uint32_t)0);
    std::cout << "close motor and sensor FD!" << std::endl;
    close(ambotMcuFd);
    close(sensorFd);
    return 0;
}

/**  
*   @brief      set position for all motor
    Parameters:
*   @param      position	[in]the quote of position for all motor
*   @return     true:get successful   false:get failed
    */
bool AmbotDriver::setMotorPosition(const std::vector<float>& position)
{
	return setAllMotorOneFeature(ControlCommandCode.SET_POSITION, position);
}

/**  
*   @brief      set position for all motor
    Parameters:
*   @param      position	[in]the quote of position for all motor
*   @return     true:get successful   false:get failed
    */
bool AmbotDriver::setMotorLocomotionCommand(const ambot_msgs::RobotAction& command)
{
    constexpr uint8_t posIndex = 0, velIndex = 2, curIndex = 4, KpIndex = 6, KdIndex = 8;
	assert(command.motorAction.size() > 0);
    assert(command.motorAction.size() >= motorNum);
    memset(writeBuffer, 0, WRITE_BUFFER_SIZE);
    writeBuffer[ControlIndex.FRAME_HEAD] = FRAME_HEAD_DATA;
    writeBuffer[ControlIndex.FUNCTION_CODE] = FunctionCode.CONTROL;
    writeBuffer[ControlIndex.COMMAND_CODE] = ControlCommandCode.SET_LOCOMOTION_CONTROL;
    writeBuffer[ControlIndex.MOTOR_NUM] = motorNum;
    dataLength = ControlIndex.DATA_LENGTH;

    for (size_t i = 0; i < motorNum; i++)
    {
        writeBuffer[ControlIndex.MOTOR_ID + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i] = motorID[i];
        // printf("id:%d, position:%f\n",  motorID[i], command.motorAction[i].q);
        floatToUint16(command.motorAction[motorID[i] - 1].q, motorLimit.minPosition, motorLimit.maxPosition,  &writeBuffer[ControlIndex.MOTOR_DATA + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i + posIndex]);
        floatToUint16(command.motorAction[motorID[i] - 1].dq, motorLimit.minVelocity, motorLimit.maxVelocity,  &writeBuffer[ControlIndex.MOTOR_DATA + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i + velIndex]);
        floatToUint16(command.motorAction[motorID[i] - 1].tor, motorLimit.minCurrent, motorLimit.maxCurrent,  &writeBuffer[ControlIndex.MOTOR_DATA + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i + curIndex]);
        floatToUint16(command.motorAction[motorID[i] - 1].Kp, motorLimit.minKp, motorLimit.maxKp,  &writeBuffer[ControlIndex.MOTOR_DATA + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i + KpIndex]);
        floatToUint16(command.motorAction[motorID[i] - 1].Kd, motorLimit.minKd, motorLimit.maxKd,  &writeBuffer[ControlIndex.MOTOR_DATA + ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE*i + KdIndex]);
        dataLength += ControlIndex.LOCOMOTION_CONTROL_DATA_SIZE;
    }
    generateCRC(&writeBuffer[ControlIndex.FUNCTION_CODE], dataLength - 1);
    dataLength += CRC_DATA_SIZE;

    if(dataLength != txPacket())
        return false;
    // don't wait for set command response now 
    return true;
}

/**  
*   @brief      set position for all motor
    Parameters:
*   @param      current		[in]the quote of current for all motor
*   @return     true:get successful   false:get failed
    */
bool AmbotDriver::setMotorCurrent(const std::vector<float>& current)
{	
	return setAllMotorOneFeature(ControlCommandCode.SET_CURRENT, current);
}


/**  
*   @brief      get all motor id and motor num
    Parameters:
*   @param      none
*   @return     true:get successful   false:get failed
    */
bool AmbotDriver::getAllMotorInformation(void)
{
    // 1. build git all motor information request frame
    memset(writeBuffer, 0, WRITE_BUFFER_SIZE);
    writeBuffer[ControlIndex.FRAME_HEAD] = FRAME_HEAD_DATA;
    writeBuffer[ControlIndex.FUNCTION_CODE] = FunctionCode.REQUEST;
    writeBuffer[ControlIndex.COMMAND_CODE] = RequestCommandCode.GET_ID;
    dataLength = ControlIndex.DATA_LENGTH - 1;
    generateCRC(&writeBuffer[ControlIndex.FUNCTION_CODE], dataLength - 1);
    dataLength += CRC_DATA_SIZE;
    // 2.send request frame
    if(dataLength != txPacket())
        return false;
    // 3.wait for request response frame
    if(waitRequestResponse(RequestCommandCode.GET_ID))
    {
        for (int i = 0; i < motorNum; i++)
        {
            motorID[i] = readBuffer[ResponseIndex.MOTOR_ID + ResponseIndex.MOTOR_DATA_SIZE*i];
            printf("ID%d : %d\n", i, motorID[i]);
        }
        return true;
    }
    else
        return false;
}

/**  
*   @brief      get all motor one feedback, template function 
    Parameters:
*   @param      inputCommandCode    [in]input command code
*   @param      data                [in]the vector data wait to send
*   @return     true:get successful   false:get failed
    */
template<typename T> bool AmbotDriver::getAllMotorOneFeedback(const uint8_t inputCommandCode, std::vector<T>& data)
{
    std::vector<T> setData;
    setData.resize(motorNum);
    for (int i = 0; i < motorNum; i++)
        setData[i] = 0;
    createCommandFrame(FunctionCode.REQUEST, inputCommandCode, setData);
    if(dataLength != txPacket())
        return false;
    if(waitRequestResponse(inputCommandCode))
    {
        assert(data.size() >= motorNum);
        for (int i = 0; i < motorNum; i++)
            byteTransferToOther(data[i], &readBuffer[ResponseIndex.MOTOR_DATA_START + ResponseIndex.MOTOR_DATA_SIZE*i]);
        return true;
    }
    else
        return false;
}

/**  
*   @brief      get all motor position data
    Parameters:
*   @param      none
*   @return     true:get successful   false:get failed
    */
#define TEMP_READ_BUFFER_LENGTH 2000
bool AmbotDriver::getAllMotorStateFromMCU(void)
{   
    // 1. init all local variables
    uint8_t currentReadCount, alreadyReadCount, resMotorNum;
    int16_t frameHeadIndex, currentTail;
    uint8_t tempReadBuffer[TEMP_READ_BUFFER_LENGTH];
    uint8_t motorFeedbackBuffer[TEMP_READ_BUFFER_LENGTH];
    uint8_t crcAuthentication[CRC_DATA_SIZE];
    static uint16_t offLineIndex = 0;

    // 2. set init value to variables
    alreadyReadCount = 0;
    frameHeadIndex = currentTail = 0;
    const uint8_t oneMotorDataLength = 9;
    const int16_t frameDataLength = motorNum * oneMotorDataLength + CRC_DATA_SIZE + 4;

    #ifdef SHOW_READ_FEEDBACK_PERIOD
    struct timeval timeOldMotor,timeNewMotor;
    gettimeofday(&timeOldMotor, NULL);
    timeNewMotor = timeOldMotor;
    #endif

    // 3.wait for receive enough data to analysis
    while (1)
    {
        // 4. if detect ctrl+C, current thread will exit 
        if (threadStop)
        {
            printf("receive data thread exit!!\n");
            pthread_exit(NULL);
        }
        usleep(WAIT_RESPONSE_US_DELAY);
        // 5.keep current receive buffer won't break, keep circle read buffer normal
        if (currentTail + 3 * frameDataLength >= TEMP_READ_BUFFER_LENGTH)
        {
            // be careful tail index can't more than head index,if it happen, reset all index
            if(currentTail < frameHeadIndex)
            {
                frameHeadIndex = 0;
                currentTail = 0;
            }
            else{
                memcpy(tempReadBuffer, &tempReadBuffer[frameHeadIndex], currentTail - frameHeadIndex);
                currentTail -= frameHeadIndex;
                frameHeadIndex = 0;
                memset(&tempReadBuffer[currentTail], 0, TEMP_READ_BUFFER_LENGTH - currentTail);
            }
        }
        // checkout motor line
        if (tempReadBuffer[frameHeadIndex + ControlIndex.FRAME_HEAD] == FRAME_HEAD_DATA && 
            tempReadBuffer[frameHeadIndex + ControlIndex.FUNCTION_CODE] == FunctionCode.REQUEST_RESPOND && 
            tempReadBuffer[frameHeadIndex + ControlIndex.COMMAND_CODE] == RequestCommandCode.MOTOR_EXCEPTION)
        {
            // do not copy the frame head
            memcpy(motorFeedbackBuffer, &tempReadBuffer[frameHeadIndex + 1], motorExceptionIndex.ALL_DATA_NUM);
            memcpy(crcAuthentication, &tempReadBuffer[frameHeadIndex + 1 + motorExceptionIndex.ALL_DATA_NUM], CRC_DATA_SIZE);
            generateCRC(motorFeedbackBuffer, motorExceptionIndex.ALL_DATA_NUM);
            frameHeadIndex += motorExceptionIndex.ALL_DATA_NUM + 1;

            if (!memcmp(&motorFeedbackBuffer[motorExceptionIndex.ALL_DATA_NUM], crcAuthentication, CRC_DATA_SIZE))
            {
                offLineIndex = (motorFeedbackBuffer[motorExceptionIndex.OFFLINE_MOTOR_HIGH] << 8) | 
                                motorFeedbackBuffer[motorExceptionIndex.OFFLINE_MOTOR_LOW];
                std::cout << COUT_RED << "WARNING:MOTOR OFFLINE, Here are offline motor index:" << std::endl;
                for (int i = 1; i <= 12; i++)
                {
                    if(offLineIndex & (1<<i))
                        std::cout << i << std::endl;
                }
                std::cout << "Please checkout these motor's connect line! and program will end!"  << COUT_RESET << std::endl;
                threadStop = true;
            }
        }
        
        // 6.read data and analysis
        currentReadCount = read(ambotMcuFd, &tempReadBuffer[currentTail], TEMP_READ_BUFFER_LENGTH - currentTail);
        currentTail += currentReadCount;
        if (currentTail - frameHeadIndex >= frameDataLength)
        {
            for (int i = frameHeadIndex; i < currentTail; i++)
            {
                if (tempReadBuffer[i + ControlIndex.FRAME_HEAD] == FRAME_HEAD_DATA && 
                    tempReadBuffer[i + ControlIndex.FUNCTION_CODE] == FunctionCode.REQUEST_RESPOND)
                {
                    frameHeadIndex = i;
                    break;
                }
            }
            // do not copy the frame head
            memcpy(motorFeedbackBuffer, &tempReadBuffer[frameHeadIndex + 1], frameDataLength - 1);           //-1:dont need the frame head
            resMotorNum = motorFeedbackBuffer[ResponseIndex.MOTOR_NUM];
            memcpy(crcAuthentication, &tempReadBuffer[frameHeadIndex + frameDataLength - CRC_DATA_SIZE], CRC_DATA_SIZE);
            generateCRC(motorFeedbackBuffer, frameDataLength - 1 - CRC_DATA_SIZE);
            frameHeadIndex += frameDataLength;

            if (!memcmp(&motorFeedbackBuffer[frameDataLength - 1 - CRC_DATA_SIZE], crcAuthentication, CRC_DATA_SIZE) && 
                (motorFeedbackBuffer[ResponseIndex.COMMAND_CODE] = RequestCommandCode.GET_POSITION))
            {
                #ifdef SHOW_READ_FEEDBACK_PERIOD
                gettimeofday(&timeNewMotor, NULL);
                printf("motor feedback period time:%ld\n", (timeNewMotor.tv_sec*1000000 + timeNewMotor.tv_usec) - (timeOldMotor.tv_sec*1000000 + timeOldMotor.tv_usec));
                timeOldMotor = timeNewMotor;
                #endif
                assert(resMotorNum <= motorNum); //if the motor num is inconsistency, program stop
                for (int i = 0; i < motorNum; i++)
                {   
                    allMotorStateAnalysis(&motorFeedbackBuffer[ResponseIndex.MOTOR_DATA_START + oneMotorDataLength*i], ambotState.motorState.at(motorID[i] - 1));
                }
                ambotState.motor_num = motorNum;
                resMotorNum = 0; 
                memset(crcAuthentication, 0 , CRC_DATA_SIZE);
            }
        }
    }
}

/**  
*   @brief      get all sensor data, include imu and force sensor touch 
    Parameters:
*   @param      none
*   @return     true:get successful   false:get failed
    */
bool AmbotDriver::getSensorDataFromMCU(void)
{   
    // 1. init all local variables
    uint8_t currentReadCount;
    uint16_t frameHeadIndex, currentTail;
    uint8_t tempReadBuffer[TEMP_READ_BUFFER_LENGTH];
    uint8_t sensorReadBuffer[READ_BUFFER_SIZE];
    uint8_t crcAuthentication[CRC_DATA_SIZE];

    // 2. set init value to variables
    frameHeadIndex = currentTail = 0;
    const uint16_t frameDataLength = sensorDataIndex.ALL_DATA_NUM + 1; //add a frameHead
    
    #ifdef SHOW_READ_FEEDBACK_PERIOD
    struct timeval timeOldSensor,timeNewSensor;
    gettimeofday(&timeNewSensor, NULL);
    timeOldSensor = timeNewSensor;
    #endif

    // 3.wait for receive enough data to analysis
    while (1)
    {
        // 4. if detect ctrl+C, current thread will exit 
        if (threadStop)
        {
            printf("sensor data receive thread exit!!\n");
            pthread_exit(NULL);
        }
        usleep(WAIT_RESPONSE_US_DELAY);
        // 5.keep current receive buffer won't break, keep circle read buffer normal
        if (currentTail + 2 * frameDataLength >= TEMP_READ_BUFFER_LENGTH)
        {
            if(currentTail < frameHeadIndex)
            {
                frameHeadIndex = 0;
                currentTail = 0;
            }
            else{
                memcpy(tempReadBuffer, &tempReadBuffer[frameHeadIndex], currentTail - frameHeadIndex);
                currentTail -= frameHeadIndex;
                frameHeadIndex = 0;
                memset(&tempReadBuffer[currentTail], 0, TEMP_READ_BUFFER_LENGTH - currentTail);
            }
        }
        // 6.read data and analysis
        currentReadCount = read(sensorFd, &tempReadBuffer[currentTail], TEMP_READ_BUFFER_LENGTH - currentTail);
        currentTail += currentReadCount;
        if (currentTail - frameHeadIndex >= frameDataLength)
        {
            for (int i = frameHeadIndex; i < currentTail; i++)
            {
                if (tempReadBuffer[i] == FRAME_HEAD_DATA && 
                    tempReadBuffer[i + ControlIndex.FUNCTION_CODE] == FunctionCode.REQUEST_RESPOND)
                {
                    frameHeadIndex = i;
                    break;
                }
            }
            // do not copy the frame head
            memcpy(sensorReadBuffer, &tempReadBuffer[frameHeadIndex + 1], frameDataLength - 1);             //-1:dont need the frame head
            memcpy(crcAuthentication, &tempReadBuffer[frameHeadIndex + 1 + sensorDataIndex.CRC_AUTHENTICATION], CRC_DATA_SIZE);
            generateCRC(sensorReadBuffer, sensorDataIndex.CRC_AUTHENTICATION);
            frameHeadIndex += frameDataLength;
            if (!memcmp(&sensorReadBuffer[sensorDataIndex.CRC_AUTHENTICATION], crcAuthentication, CRC_DATA_SIZE) && 
                (sensorReadBuffer[sensorDataIndex.COMMAND_CODE] = RequestCommandCode.GET_SENSOR_IMU))
            {
                #ifdef SHOW_READ_FEEDBACK_PERIOD
                gettimeofday(&timeNewSensor, NULL);
                printf("sensor feedback period time:%ld\n", (timeNewSensor.tv_sec*1000000 + timeNewSensor.tv_usec) - (timeOldSensor.tv_sec*1000000 + timeOldSensor.tv_usec));
                timeOldSensor = timeNewSensor;
                #endif
                byteTransferToOther(ambotState.imu.acceleration.x, &sensorReadBuffer[sensorDataIndex.IMU_LINE_ACC_X]);
                byteTransferToOther(ambotState.imu.acceleration.y, &sensorReadBuffer[sensorDataIndex.IMU_LINE_ACC_Y]);
                byteTransferToOther(ambotState.imu.acceleration.z, &sensorReadBuffer[sensorDataIndex.IMU_LINE_ACC_Z]);
                byteTransferToOther(ambotState.imu.gyroscope.x, &sensorReadBuffer[sensorDataIndex.IMU_ANGULAR_X]);
                byteTransferToOther(ambotState.imu.gyroscope.y, &sensorReadBuffer[sensorDataIndex.IMU_ANGULAR_Y]);
                byteTransferToOther(ambotState.imu.gyroscope.z, &sensorReadBuffer[sensorDataIndex.IMU_ANGULAR_Z]);
                ambotState.imu.gyroscope.x*=(3.1415926/180);
                ambotState.imu.gyroscope.y*=(3.1415926/180);
                ambotState.imu.gyroscope.z*=(3.1415926/180);
                byteTransferToOther(ambotState.imu.quaternion.w, &sensorReadBuffer[sensorDataIndex.IMU_QUATERNION_W]);
                byteTransferToOther(ambotState.imu.quaternion.x, &sensorReadBuffer[sensorDataIndex.IMU_QUATERNION_X]);
                byteTransferToOther(ambotState.imu.quaternion.y, &sensorReadBuffer[sensorDataIndex.IMU_QUATERNION_Y]);
                byteTransferToOther(ambotState.imu.quaternion.z, &sensorReadBuffer[sensorDataIndex.IMU_QUATERNION_Z]);
                ambotState.force.touch.FL = sensorReadBuffer[sensorDataIndex.TOUCH_FL];
                ambotState.force.touch.FR = sensorReadBuffer[sensorDataIndex.TOUCH_FR];
                ambotState.force.touch.HL = sensorReadBuffer[sensorDataIndex.TOUCH_HL];
                ambotState.force.touch.HR = sensorReadBuffer[sensorDataIndex.TOUCH_HR];
                ambotState.force.shift.FL = sensorReadBuffer[sensorDataIndex.SHIFT_FL];
                ambotState.force.shift.FR = sensorReadBuffer[sensorDataIndex.SHIFT_FR];
                ambotState.force.shift.HL = sensorReadBuffer[sensorDataIndex.SHIFT_HL];
                ambotState.force.shift.HR = sensorReadBuffer[sensorDataIndex.SHIFT_HR];
                memset(crcAuthentication, 0 , CRC_DATA_SIZE);
            }
        }
    }
}

/**  
*   @brief      the data transfer function: float to byte
    Parameters:
*   @param      value    	[in]the reference of goal value 
*   @param      buffer 		[in]the pointer of wait convert 
*   @return     none
    */
template<typename T> void AmbotDriver::otherTransferToByte(const T value, uint8_t* buffer)
{
    T temp = value;
    uint8_t* point = (uint8_t*)&temp;
    for (int i = 0; i < sizeof(T); i++)
        buffer[i] = *(point+i);
}

/**  
*   @brief      the data transfer function: float to byte
    Parameters:
*   @param      goalValue    	[in]the reference of goal value 
*   @param      buffer 		    [in]the pointer of wait convert 
*   @return     none
    */
template<typename T> void AmbotDriver::byteTransferToOther(T& goalValue, const uint8_t* buffer)
{
    //T temp;
    float temp;
    uint8_t* point = (uint8_t*)&temp;
    //std::cout<<"size:"<<sizeof(float)<<std::endl;
    for (int i = 0; i < sizeof(float); i++)
        *(point + i) = *(buffer + i);
    goalValue = temp;
}

/**  
*   @brief      the data transfer function: uint16 to float and return data
    Parameters:
*   @param      input    	[in]input data, wait to transfer
*   @param      min 	    [in]the min limit of transfer data 
*   @param      max    	    [in]the max limit of transfer data 
*   @return     the transfer data
    */
float AmbotDriver::uint16ToFloat(uint16_t input, float min, float max)
{
    return (float)((max - min) * input / UINT16_MAX) + min;
}

/**  
*   @brief      the data transfer function: float to uint16 and save to pointer
    Parameters:
*   @param      input    	[in]input data, wait to transfer
*   @param      min 	    [in]the min limit of transfer data 
*   @param      max    	    [in]the max limit of transfer data 
*   @param      des 	    [in]the destination pointer of save addr
*   @return     none
    */
void AmbotDriver::floatToUint16(float input, float min, float max, uint8_t* des)
{
    uint16_t result;
    if (input > max)
        result = UINT16_MAX;
    else if (input < min)
        result = 0;
    else
        result = (uint16_t)(UINT16_MAX * (input - min)/(max - min));
    uint16ToPoint(result, des);
}

void AmbotDriver::uint16ToPoint(uint16_t input, uint8_t* des)
{
    *des = input >> 8;
    *(des + 1) = input;
}
