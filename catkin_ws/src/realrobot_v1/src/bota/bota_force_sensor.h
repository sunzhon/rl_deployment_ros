/* A class to read force values of Bota MiniOne sensors
 *
 * Author： chen chen,wang tian ao
 * Date: 2024-1-19
 * Email: 1240563221@qq.com
 *
 */

#pragma once
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include "pthread.h"
#include "ros/ros.h"
#include "serial/serial.h"
#include "assert.h"
#include <vector>
using namespace std;


namespace bota_ns{

	typedef struct{
		int status;
		float fx;
		float fy;
		float fz;
		float mx;
		float my;
		float mz;
		long int timestamp;
		float temperature;
	} ForceInfo;


	enum MiniOneConf
	{
		BOTA_PRODUCT_CODE = 123456,
		BAUDERATE = 460800,
		SINC_LENGTH = 512,
		CHOP_ENABLE = 0,
		FAST_ENABLE = 0,
		FIR_DISABLE = 1,
		TEMP_COMPENSATION = 0, //# 0: Disabled (recommended), 1: Enabled
		USE_CALIBRATION = 1, //# 1: calibration matrix active, 0: raw measurements
		DATA_FORMAT = 0, // # 0: binary, 1: CSV
		BAUDERATE_CONFIG = 4, //# 0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800
		FRAME_HEADER = 0xaa,
	};



	class BotaMiniOne{
		public:
			explicit BotaMiniOne(std::string deviceName, int baudRate);
			~BotaMiniOne();
			bool openDevice(string device, int baudrate);
			bool setUpDevice();
			void readForce(void);
			static void *newReadThread(void* arg);
			float transformRxFloat(uint8_t *BUF, uint16_t start_address);
			short transformRxShort(uint8_t *BUF, uint16_t start_address);
			long int transformRxInt(uint8_t *buf, uint16_t start_address);
			inline uint16_t calcCrc16X25(uint8_t* data, int len);
			uint16_t crcCcittUpdate(uint16_t crc, uint8_t data);
			ForceInfo getBotaData();
			bool getReadThreadFlag(void);

		private:
			pthread_t tid;
			bool readThreadFlag = false;
			ForceInfo force_info;
			serial::Serial* forceSerial;
	};

	#define SENSOR_DATA_NUM 6
	#define MEAN_FILTER_WINDOW 15
	class BotaSensor{
		public:
			explicit BotaSensor();
			~BotaSensor();
			void initSensor(const std::vector<std::string>& deviceName, const int baudRate);
			void setSensorData(void);
			bool getSensorInitState(void);
			void getSensorData(std::vector<float>& data);
		private:
			bool sensorInitFlag = false;
			int forceSensorNum;
			void meanFilter();
			std::vector<float> forceSensorData;
			std::vector<vector<float>> meanFilterBuffer;
			std::vector<BotaMiniOne*> forceSensor;
			std::vector<float> forceSensorDataOffset;
	};
}
