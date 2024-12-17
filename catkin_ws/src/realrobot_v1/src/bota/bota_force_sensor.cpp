/* A class to read force values of Bota MiniOne sensors
*
* Author： chen chen,wang tian ao
* Date: 2024-1-19
* Email: 1240563221@qq.com
*
	*/


#include "bota_force_sensor.h"
// #include "unistd.h"
// using namespace std;

namespace bota_ns{
	/**  
	*   @brief      new thread to update bota force sensor data
		Parameters:
	*   @param      arg   	[in]this is a pointer to deliver parameter to thread
	*   @return     none  
		*/
	void *BotaMiniOne::newReadThread(void* arg)
	{
		BotaMiniOne* ptr = (bota_ns::BotaMiniOne*) arg;
		ptr->readForce();
		pthread_exit(0);
	}

	/**  
	*   @brief      this is construct function of BotaMiniOne Class
		Parameters:
	*   @param      deviceName      [in]the device name of device node
	*   @param      baudRate 		[in]the communication baud rate you want
	*   @return     none    
		*/
	BotaMiniOne::BotaMiniOne(std::string deviceName, int baudRate)
	{
		forceSerial = new serial::Serial;
		printf("bota mini one force sensor!\r\n");

		
		force_info.status = 0;
		force_info.fx = 0.0;
		force_info.fy = 0.0;
		force_info.fz = 0.0;
		force_info.mx = 0.0;
		force_info.my = 0.0;
		force_info.mz = 0.0;
		force_info.timestamp = 0;
		force_info.temperature = 0.0;
		if(openDevice(deviceName, baudRate))
		{
			if(pthread_create(&tid, NULL, newReadThread, (void *)this) != 0){ 
				perror("Create read force sensor data thread fail in controller!\n");
			}else{
				readThreadFlag = true;
				perror("Create read force sensor data thread successful!\n");
			}
		}
	}

	/**  
	*   @brief      this is deconstruct function of BotaMiniOne Class
		Parameters:
	*   @return     none    
		*/
	BotaMiniOne::~BotaMiniOne(){
		forceSerial->close();
		delete forceSerial;
	}

	/**  
	*   @brief      open device node in linux system
		Parameters:
	*   @param      deviceName      [in]the device name of device node
	*   @param      baudrate 		[in]the communication baud rate you want
	*   @return     none    
		*/
	bool BotaMiniOne::openDevice(std::string device, int baudrate){
		if(access(device.c_str(), F_OK) == 0){
			try 
			{ 
				// set up the serial port and open it
				forceSerial->setPort(device);
				forceSerial->setBaudrate(baudrate);
				serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
				forceSerial->setTimeout(to); 
				forceSerial->open(); 
			} 
			catch (serial::IOException& e) 
			{ 
				perror("Unable to open port: Mini one\r\n"); 
				return false; 
			}    
			// check the whether the serial port is opened 
			if(forceSerial->isOpen()) 
			{ 
				printf("Separate MiniOne serial Port initial successful!\r\n"); 
				forceSerial->flushInput();
				sleep(0.1);
				setUpDevice();
				return true;
			} 
			else 
			{ 
				perror("Open separate Minione seiral port fail!");
				return false; 
			}
		}
		else{
			perror("Did not find bota minione device");
			return false;

		}
		return true;

	}

	/**  
	*   @brief      make init setup for force sensor 
		Parameters:
	*   @return     none    
		*/
	bool BotaMiniOne::setUpDevice(){
		printf("Trying to setup Mini one\r\n");	

		sleep(0.5);
		forceSerial->flushInput();
		forceSerial->flushOutput();

		//go to config mode
		uint8_t  cmd = 'C';
		forceSerial->write(&cmd, sizeof(cmd));

		/*	Communication setup
			c: config mode 
			temperature compensation --- 0:disable  ---- 1:enable
			calibration ---- 1:calibration matrix active --- 0: raw measurements
			data format ---- 0:binary --- 1: CSV
			baudrate config ---- 0: 9600 -- 1: 57600 -- 2: 115200 -- 3: 230400 -- 4: 460800
		*/
		uint8_t comm_setup[] = "c,0,1,0,4";
		forceSerial->write(comm_setup, sizeof(comm_setup));

		//set the sample rate, calculate by the SINC_LENGTH
		double time_step = 0.00001953125 * SINC_LENGTH;
		printf("Timestep:%f \r\n", time_step);

		//Filter setup
		uint8_t filter_setup[] = "512,0,0,1";
		forceSerial->write(filter_setup, sizeof(filter_setup));

		//Go to RUN mode
		cmd = 'R';
		forceSerial->write(&cmd, sizeof(cmd));

		return true;
	}

	/**  
	*   @brief      get the result of crc calculate
		Parameters:
	*   @param      data      	[in]the pointer of waiting for calculate crc of data
	*   @param      len			[in]data length	
	*   @return     crc result    
		*/
	inline uint16_t BotaMiniOne::calcCrc16X25(uint8_t* data, int len)
	{
		uint16_t crc = 0xFFFF;
		while (len--)
			crc = crcCcittUpdate(crc, *data++);
		return ~crc;
	}

	/**  
	*   @brief      authentication crc and data(I am not sure)
		Parameters:
	*   @param      data      	[in]the pointer of waiting for calculate crc of data
	*   @param      len			[in]data length	
	*   @return     crc result    
		*/
	uint16_t BotaMiniOne::crcCcittUpdate(uint16_t crc, uint8_t data)
	{
		#define lo8(x) ((x)&0xFF)
		#define hi8(x) (((x) >> 8) & 0xFF)
		data ^= lo8(crc);
		data ^= data << 4;
		return ((((uint16_t)data << 8) | hi8(crc)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
	}
	
	/**  
	*   @brief      read data - roll
		Parameters:
	*   @param      
	*   @return     none
		*/
	void BotaMiniOne::readForce(void){

		uint8_t  results[34];
		uint8_t  framhead;

		uint8_t  crcdata[2];
		uint16_t crc16_ccitt_frame;

		
		size_t n = forceSerial->available();

		while (1)
		{
			usleep(20);
			if(forceSerial->available())
			{
				forceSerial->read(&framhead, 1);
				if(framhead == FRAME_HEADER)
				{
					// printf("get frame head 0xAA\r\n");

					memset(results,  0,  sizeof(results));
					memset(crcdata, 0, sizeof(crcdata));
					
					forceSerial->read(results, 34);					//数据包读取
					forceSerial->read(crcdata, 2);					//crc读取

					crc16_ccitt_frame = ((crcdata[1]<<8)&0xFF00)  | ( crcdata[0] & 0xFF) ;

					if(crc16_ccitt_frame == calcCrc16X25(results, sizeof(results)))
					{
						force_info.status = transformRxShort(results, 0);
						force_info.fx =  transformRxFloat(results, 2);
						force_info.fy = transformRxFloat(results, 6);
						force_info.fz = transformRxFloat(results, 10);
						force_info.mx = transformRxFloat(results, 14);
						force_info.my = transformRxFloat(results, 18);
						force_info.mz = transformRxFloat(results, 22);
						force_info.timestamp = transformRxInt(results, 26);
						force_info.temperature = transformRxFloat(results, 30);
						// printf("Run my loop\r\n");
						// printf("Status %d\r\n",force_info.status);
						// printf("Fx %f\r\n", force_info.fx);
						// printf("Fy %f\r\n", force_info.fy);
						// printf("Fz %f\r\n", force_info.fz);
						// printf("Mx %f\r\n", force_info.mx);
						// printf("My %f\r\n", force_info.my);
						// printf("Mz %f\r\n", force_info.mz);
						// printf("Timestamp %d\r\n", force_info.timestamp);
						// printf("Temperature %f\r\n", force_info.temperature);
					}
					else{
						// printf("CRC mismatch received\r\n");
					}				
				}
			}			
		}
    }

	/**  
	*   @brief      uint8_t data transform to float
		Parameters:
	*   @param      buf      		[in]the pointer of waiting for transform
	*   @param      start_address	[in]start address
	*   @return     transform result  
		*/
	float BotaMiniOne::transformRxFloat(uint8_t *buf, uint16_t start_address)
	{
		union transform
		{
			float fdata;
			uint8_t uchar[4];
		}trans;
		trans.uchar[0]=*(buf+start_address);
		trans.uchar[1]=*(buf+start_address+1);
		trans.uchar[2]=*(buf+start_address+2);
		trans.uchar[3]=*(buf+start_address+3);
		return trans.fdata;
	}

	/**  
	*   @brief     uint8_t data transform to int
		Parameters:
	*   @param      buf      		[in]the pointer of waiting for transform
	*   @param      start_address	[in]start address
	*   @return     transform result  
		*/
	long int BotaMiniOne::transformRxInt(uint8_t *buf, uint16_t start_address)
	{
		union transform
		{
			long int fdata;
			uint8_t uchar[4];
		}trans;
		trans.uchar[0]=*(buf+start_address);
		trans.uchar[1]=*(buf+start_address+1);
		trans.uchar[2]=*(buf+start_address+2);
		trans.uchar[3]=*(buf+start_address+3);
		return trans.fdata;
	}

	/**  
	*   @brief     uint8_t data transform to short
		Parameters:
	*   @param      buf      		[in]the pointer of waiting for transform
	*   @param      start_address	[in]start address
	*   @return     transform result  
		*/
	short BotaMiniOne::transformRxShort(uint8_t *buf, uint16_t start_address)
	{
		union transform
		{
			short fdata;
			uint8_t uchar[2];
		}trans;
		trans.uchar[0]=*(buf+start_address);
		trans.uchar[1]=*(buf+start_address+1);
		return trans.fdata;
	}

	/**  
	*   @brief     get the newest data 
		Parameters:
	*   @param      
	*   @return     the newest force data 
		*/
	ForceInfo BotaMiniOne::getBotaData()
	{
		return force_info;
	}

	/**  
	*   @brief     make sure whether the thread is running
		Parameters:
	*   @param      
	*   @return    running flag
		*/  	
	bool BotaMiniOne::getReadThreadFlag(void)
	{
		return readThreadFlag;
	}


/********************** other class ************************/
/********************** other class ************************/
/********************** other class ************************/
/********************** other class ************************/
/********************** other class ************************/
	/**  
	*   @brief     the BotaSensor class construct function
		Parameters:
	*   @param     
	*   @return    none
		*/  
	BotaSensor::BotaSensor()
	{
	} 

	/**  
	*   @brief     the BotaSensor class construct function
		Parameters:
	*   @param     deviceName 	[in] all device name 
	*   @param     baudRate 	[in] baudrate you want
	*   @return    none
		*/  
	void BotaSensor::initSensor(const std::vector<std::string>& deviceName, const int baudRate)
	{
		forceSensorNum = deviceName.size();
		forceSensorData.resize(forceSensorNum*SENSOR_DATA_NUM);
		meanFilterBuffer.resize(MEAN_FILTER_WINDOW);
		for (int i =0; i < MEAN_FILTER_WINDOW; i++)
		{
			meanFilterBuffer.at(i).resize(forceSensorNum*SENSOR_DATA_NUM);
		}
		forceSensor.resize(forceSensorNum);
		for (int i = 0; i < forceSensorNum; i++)
		{
			forceSensor.at(i) = new BotaMiniOne(deviceName[i], baudRate);
		}
		forceSensorDataOffset.resize(forceSensorNum*SENSOR_DATA_NUM);
		forceSensorDataOffset[0] = 17.348;
		forceSensorDataOffset[1] = -11.4466;
		forceSensorDataOffset[2] = -5.1314;
		forceSensorDataOffset[6] = -26.6548;
		forceSensorDataOffset[7] = -10.3249;
		forceSensorDataOffset[8] = 2.8087;
		forceSensorDataOffset[12] = 10.0947;
		forceSensorDataOffset[13] = -11.3604;
		forceSensorDataOffset[14] = 0.717938;
		forceSensorDataOffset[18] = -14.0075;
		forceSensorDataOffset[19] = -9.60094;
		forceSensorDataOffset[20] = -2.83329;
	} 

	/**  
	*   @brief     the BotaSensor class deconstruct function
		Parameters:
	*   @param     
	*   @return    none
		*/ 
	BotaSensor::~BotaSensor()
	{
		for (int i = 0; i < forceSensorNum; i++)
		{
			delete forceSensor.at(i);
		}
	}

	/**  
	*   @brief     make sure whether all force sensor is ok
		Parameters:
	*   @param     
	*   @return    ok:true    not ok:false
		*/ 
	bool BotaSensor::getSensorInitState(void)
	{
		for (int i = 0; i < forceSensorNum; i++)
		{
			if (!forceSensor.at(i)->getReadThreadFlag())
			{
				return false;
			}
		}
		return true;
	}

	/**  
	*   @brief     set all sensor data 
		Parameters:
	*   @param     
	*   @return    none
		*/ 
	void BotaSensor::setSensorData(void)
	{
		ForceInfo oneForceData;
		for (int i = 0; i < forceSensorNum; i++)
		{
			oneForceData = forceSensor.at(i)->getBotaData();
			forceSensorData.at(i*SENSOR_DATA_NUM + 0) = oneForceData.fx;
			forceSensorData.at(i*SENSOR_DATA_NUM + 1) = oneForceData.fy;
			forceSensorData.at(i*SENSOR_DATA_NUM + 2) = oneForceData.fz;
			forceSensorData.at(i*SENSOR_DATA_NUM + 3) = oneForceData.mx;
			forceSensorData.at(i*SENSOR_DATA_NUM + 4) = oneForceData.my;
			forceSensorData.at(i*SENSOR_DATA_NUM + 5) = oneForceData.mz;
			// printf("id:%d %f  %f  %f  %f  %f  %f\n", i, oneForceData.fx, oneForceData.fy,
			// 		oneForceData.fz, oneForceData.mx, oneForceData.my, oneForceData.mz);
		}
	}
	/**
	*	@brief  meanFilter
		Parameters:
	*	@param
	*	@return		none
	*/
	void BotaSensor::meanFilter()
	{
		static uint8_t index=0;
		for (int i=0; i<forceSensorNum*SENSOR_DATA_NUM; i++)
		{
			meanFilterBuffer.at(index).at(i)=forceSensorData.at(i)-forceSensorDataOffset.at(i);
		}
		for (int i=0; i<forceSensorNum*SENSOR_DATA_NUM; i++)
		{
			float sum = 0;
			for (int j=0; j<MEAN_FILTER_WINDOW; j++)
			{
				sum += meanFilterBuffer.at(j).at(i);
			}
			forceSensorData.at(i)=sum/MEAN_FILTER_WINDOW;
		}
		index++;
		if (index>MEAN_FILTER_WINDOW-1) 
			index=0;
	}
	/**  
	*   @brief     get all the sensor data
		Parameters:
	*   @param     data 	[out] the quote of all the data 
	*   @return    none
		*/ 
	void BotaSensor::getSensorData(std::vector<float>& data)
	{
		assert(data.size() >= forceSensorData.size());
		setSensorData();
		meanFilter();
		for (int i = 0; i < forceSensorData.size(); i++)
			data.at(i) = forceSensorData.at(i);
	}
}     


