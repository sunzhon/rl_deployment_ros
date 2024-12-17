/*
 * Copyright [2015] [Ke Sun]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "imu_vn_100.h"
#include "vn_errorCodes.h"
#include "unistd.h"
namespace imu_vn_100 {

// LESS HACK IS STILL HACK
ImuVn100* imu_vn_100_ptr;

using sensor_msgs::Imu;
using sensor_msgs::MagneticField;
using sensor_msgs::FluidPressure;
using sensor_msgs::Temperature;

void RosVector3FromVnVector3(geometry_msgs::Vector3& ros_vec3,
                             const VnVector3& vn_vec3);
void RosQuaternionFromVnQuaternion(geometry_msgs::Quaternion& ros_quat,
                                   const VnQuaternion& vn_quat);
void FillImuMessage(sensor_msgs::Imu& imu_msg,
                    const VnDeviceCompositeData& data, bool binary_output);

void AsyncListener(void* sender, VnDeviceCompositeData* data) {
  //imu_vn_100_ptr->PublishData(*data);
  imu_vn_100_ptr->updateImuData(*data);
}

constexpr int ImuVn100::kBaseImuRate;
constexpr int ImuVn100::kDefaultImuRate;
constexpr int ImuVn100::kDefaultSyncOutRate;

void ImuVn100::SyncInfo::Update(const unsigned sync_count,
                                const ros::Time& sync_time) {
  if (rate <= 0) return;

  if (count != sync_count) {
    count = sync_count;
    time = sync_time;
  }
}

bool ImuVn100::SyncInfo::SyncEnabled() const { return rate > 0; }

void ImuVn100::SyncInfo::FixSyncRate() {
  // Check the sync out rate
  if (SyncEnabled()) {
    if (ImuVn100::kBaseImuRate % rate != 0) {
      rate = ImuVn100::kBaseImuRate / (ImuVn100::kBaseImuRate / rate);
      printf("Set SYNC_OUT_RATE to %d\n", rate);
    }
    skip_count =
        (std::floor(ImuVn100::kBaseImuRate / static_cast<double>(rate) +
                    0.5f)) -
        1;

    if (pulse_width_us > 10000) {
      printf("Sync out pulse with is over 10ms. Reset to 1ms\n");
      pulse_width_us = 1000;
    }
    rate_double = rate;
  }

  printf("Sync out rate: %d\n", rate);
}

ImuVn100::ImuVn100()
    : port_(std::string("/dev/ttyUSB1")),
      baudrate_(921600),
      frame_id_(std::string("imu")) {
  // Initialize();
  // time_start_ = ros::Time::now();
  imu_vn_100_ptr = this;
}

ImuVn100::~ImuVn100() { Disconnect(); }

void ImuVn100::FixImuRate() {
  if (imu_rate_ <= 0) {
    printf("Imu rate %d is < 0. Set to %d\n", imu_rate_, kDefaultImuRate);
    imu_rate_ = kDefaultImuRate;
  }

  if (kBaseImuRate % imu_rate_ != 0) {
    int imu_rate_old = imu_rate_;
    imu_rate_ = kBaseImuRate / (kBaseImuRate / imu_rate_old);
    printf("Imu rate %d cannot evenly decimate base rate %d, reset to %d\n",
             imu_rate_old, kBaseImuRate, imu_rate_);
  }
}

void ImuVn100::LoadParameters(const std::string& deviceName,const int& baudRate) {
  port_ = deviceName;
  baudrate_ = baudRate;
  imu_rate_ = kDefaultImuRate;
  sync_info_.rate = kDefaultSyncOutRate;
  sync_info_.pulse_width_us = 1000;
  binary_output_ = true;
  serial_interface_number_ = 1;

  // pnh_.param<std::string>("port", port_, deviceName);
  // pnh_.param<std::string>("frame_id", frame_id_, pnh_.getNamespace());
  // pnh_.param("baudrate", baudrate_, baudRate);
  // pnh_.param("imu_rate", imu_rate_, kDefaultImuRate);

  // pnh_.param("enable_mag", enable_mag_, false);
  // pnh_.param("enable_pres", enable_pres_, false);
  // pnh_.param("enable_temp", enable_temp_, false);

  // pnh_.param("sync_rate", sync_info_.rate, kDefaultSyncOutRate);
  // pnh_.param("sync_pulse_width_us", sync_info_.pulse_width_us, 1000);

  // pnh_.param("binary_output", binary_output_, true);
  // pnh_.param("serial_interface_number",serial_interface_number_ , 1);

  FixImuRate();
  sync_info_.FixSyncRate();
}

void ImuVn100::CreateDiagnosedPublishers() {
  imu_rate_double_ = imu_rate_;
  //pd_imu_.Create<Imu>(pnh_, "imu", updater_, imu_rate_double_);
  // if (enable_mag_) {
  //   pd_mag_.Create<MagneticField>(pnh_, "magnetic_field", updater_,
  //                                 imu_rate_double_);
  // }
  // if (enable_pres_) {
  //   pd_pres_.Create<FluidPressure>(pnh_, "fluid_pressure", updater_,
  //                                  imu_rate_double_);
  // }
  // if (enable_temp_) {
  //   pd_temp_.Create<Temperature>(pnh_, "temperature", updater_,
  //                                imu_rate_double_);
  // }

}

bool ImuVn100::Initialize(const std::string& deviceName,const int& baudRate) {
  LoadParameters(deviceName, baudRate);

  printf("Connecting to device\n");
  if (VNERR_NO_ERROR != vn100_connect(&imu_, port_.c_str(), 115200))
  printf("using 115200 connected\n");
  usleep(300000);
  unsigned int old_baudrate;
  if (VNERR_NO_ERROR != vn100_getSerialBaudRate(&imu_, &old_baudrate))
  {
    printf("Reconnecting the device\n");
    vn100_disconnect(&imu_);
    if (VNERR_NO_ERROR != vn100_connect(&imu_, port_.c_str(), baudrate_))
      return false;
    printf("baudrate now 921600");
    usleep(300000);
    if(VNERR_NO_ERROR != vn100_getSerialBaudRate(&imu_, &old_baudrate))
      return false;
  }
  printf("Default serial baudrate: %u\n", old_baudrate);
  printf("Set serial baudrate to %d\n", baudrate_);
  if (VNERR_NO_ERROR != vn100_setSerialBaudRate(&imu_, baudrate_, true))
    return false;
  printf("Disconnecting the device\n");
  vn100_disconnect(&imu_);
  usleep(300000);
  printf("Reconnecting to device\n");
  // if (VNERR_NO_ERROR != vn100_connect(&imu_, port_.c_str(), baudrate_))
  if (VNERR_NO_ERROR != vn100_connect(&imu_, port_.c_str(), baudrate_))
    return false;
  usleep(300000);
  printf("Connected to device at %s\n", port_.c_str());
  if (VNERR_NO_ERROR != vn100_getSerialBaudRate(&imu_, &old_baudrate))
    return false;
  printf("New serial baudrate: %u\n", old_baudrate);


  // Idle the device for intialization
  if (VNERR_NO_ERROR != vn100_pauseAsyncOutputs(&imu_, true))
    return false;

  printf("Fetching device info.\n");
  char model_number_buffer[30] = {0};
  int hardware_revision = 0;
  char serial_number_buffer[30] = {0};
  char firmware_version_buffer[30] = {0};

  if (VNERR_NO_ERROR != vn100_getModelNumber(&imu_, model_number_buffer, 30))
    return false;
  printf("Model number: %s\n", model_number_buffer);

  if (VNERR_NO_ERROR != vn100_getHardwareRevision(&imu_, &hardware_revision))
    return false;
  printf("Hardware revision: %d\n", hardware_revision);
  if (VNERR_NO_ERROR != vn100_getSerialNumber(&imu_, serial_number_buffer, 30))
    return false;
  printf("Serial number: %s\n", serial_number_buffer);

  if (VNERR_NO_ERROR != vn100_getFirmwareVersion(&imu_, firmware_version_buffer, 30))
    return false;
  printf("Firmware version: %s\n", firmware_version_buffer);

  if (sync_info_.SyncEnabled()) {
    printf("Set Synchronization Control Register (id:32).\n");
    VnEnsure(vn100_setSynchronizationControl(
        &imu_, SYNCINMODE_COUNT, SYNCINEDGE_RISING, 0, SYNCOUTMODE_IMU_START,
        SYNCOUTPOLARITY_POSITIVE, sync_info_.skip_count,
        sync_info_.pulse_width_us * 1000, true));

    if (!binary_output_) {
      printf("Set Communication Protocal Control Register (id:30).\n");
      VnEnsure(vn100_setCommunicationProtocolControl(
          &imu_, SERIALCOUNT_SYNCOUT_COUNT, SERIALSTATUS_OFF, SPICOUNT_NONE,
          SPISTATUS_OFF, SERIALCHECKSUM_8BIT, SPICHECKSUM_8BIT, ERRORMODE_SEND,
          true));
    }
  }

  //CreateDiagnosedPublishers();

  auto hardware_id = std::string("vn100-") + std::string(model_number_buffer) +
                     std::string(serial_number_buffer);
  updater_.setHardwareID(hardware_id);
  
  std::cout << "initialize OK!\n";
  return true;
}

void ImuVn100::Stream(bool async) {
  // Pause the device first
  std::cout << "stream on.\n";
  VnEnsure(vn100_pauseAsyncOutputs(&imu_, true));

  if (async) {
    VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true));
    std::cout << "async on\n";
    if (binary_output_) {
          std::cout << "binary on\n";
          // Set the binary output data type and data rate
          int binary_async_mode;
          if(serial_interface_number_ == 1)      binary_async_mode = BINARY_ASYNC_MODE_SERIAL_1;
          else if(serial_interface_number_ == 2) binary_async_mode = BINARY_ASYNC_MODE_SERIAL_2;
          else throw std::runtime_error("VN: Unknown serial interface number!\n");

          VnEnsure(vn100_setBinaryOutput1Configuration(
          &imu_, binary_async_mode, kBaseImuRate / imu_rate_,
          BG1_TIME_STARTUP | BG1_QTN | BG1_IMU | BG1_MAG_PRES | BG1_SYNC_IN_CNT,
          BG3_NONE, BG5_NONE, true));
    } 
    else {
          // Set the ASCII output data type and data rate
          // ROS_INFO("Configure the output data type and frequency (id: 6 & 7)");
          std::cout << "ascii data\n";
          VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_VNIMU, true));
     }

    // Add a callback function for new data event
    VnEnsure(vn100_registerAsyncDataReceivedListener(&imu_, &AsyncListener));

    ROS_INFO("Setting IMU rate to %d", imu_rate_);
    VnEnsure(vn100_setAsynchronousDataOutputFrequency(&imu_, imu_rate_, true));
  }
   else {
    // Mute the stream
    ROS_DEBUG("Mute the device");
    VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true));
    // Remove the callback function for new data event
    VnEnsure(vn100_unregisterAsyncDataReceivedListener(&imu_, &AsyncListener));
  }
  // Resume the device
  VnEnsure(vn100_resumeAsyncOutputs(&imu_, true));
}

void ImuVn100::Resume(bool need_reply) {
  vn100_resumeAsyncOutputs(&imu_, need_reply);
}

void ImuVn100::Idle(bool need_reply) {
  vn100_pauseAsyncOutputs(&imu_, need_reply);
}

void ImuVn100::Disconnect() {
  // TODO: why reset the device?
  vn100_reset(&imu_);
  vn100_disconnect(&imu_);
}

sensor_msgs::Imu ImuVn100::getVn100ImuData()
{
  return imu_send;
}

void ImuVn100::PublishData(const VnDeviceCompositeData& data) {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp    = ros::Time::now();
  imu_msg.header.frame_id = frame_id_;
  //  std::cout << "rostime:" << (imu_msg.header.stamp.toSec()-time_start_.toSec()) << std::endl;;
  //  std::cout << "imutime:" << (double)data.timeStartup/1000000000.0 << std::endl;
  //  std::cout << "gap:" <<(imu_msg.header.stamp.toSec()-time_start_.toSec())-(double)data.timeStartup/1000000000.0 <<"\n\n";
  FillImuMessage(imu_msg, data, binary_output_);
  imu_send = imu_msg;                 //数据传递接口
  pd_imu_.Publish(imu_msg);

  if (enable_mag_) {
    sensor_msgs::MagneticField mag_msg;
    mag_msg.header = imu_msg.header;
    RosVector3FromVnVector3(mag_msg.magnetic_field, data.magnetic);
    pd_mag_.Publish(mag_msg);
  }

  if (enable_pres_) {
    sensor_msgs::FluidPressure pres_msg;
    pres_msg.header = imu_msg.header;
    pres_msg.fluid_pressure = data.pressure;
    pd_pres_.Publish(pres_msg);
  }

  if (enable_temp_) {
    sensor_msgs::Temperature temp_msg;
    temp_msg.header = imu_msg.header;
    temp_msg.temperature = data.temperature;
    pd_temp_.Publish(temp_msg);
  }
  //ROS_INFO("IMU update!");
  sync_info_.Update(data.syncInCnt, imu_msg.header.stamp);

  updater_.update();
}

void ImuVn100::updateImuData(const VnDeviceCompositeData& data) {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp    = ros::Time::now();
  imu_msg.header.frame_id = frame_id_;
  
  FillImuMessage(imu_msg, data, binary_output_);

  imu_send = imu_msg;                 //数据传递接口

  //ROS_INFO("IMU update!");
  sync_info_.Update(data.syncInCnt, imu_msg.header.stamp);
  updater_.update();
}

void VnEnsure(const VnErrorCode& error_code) {
  if (error_code == VNERR_NO_ERROR) return;

  switch (error_code) {
    case VNERR_UNKNOWN_ERROR:
      throw std::runtime_error("VN: Unknown error");
    case VNERR_NOT_IMPLEMENTED:
      throw std::runtime_error("VN: Not implemented");
    case VNERR_TIMEOUT:
      ROS_WARN("Opertation time out");
      break;
    case VNERR_SENSOR_INVALID_PARAMETER:
      ROS_WARN("VN: Sensor invalid paramter");
      break;
    case VNERR_INVALID_VALUE:
      ROS_WARN("VN: Invalid value");
      break;
    case VNERR_FILE_NOT_FOUND:
      ROS_WARN("VN: File not found");
      break;
    case VNERR_NOT_CONNECTED:
      throw std::runtime_error("VN: not connected");
    case VNERR_PERMISSION_DENIED:
      throw std::runtime_error("VN: Permission denied");
    default:
      ROS_WARN("Unhandled error type");
  }
}

void RosVector3FromVnVector3(geometry_msgs::Vector3& ros_vec3,
                             const VnVector3& vn_vec3) {
  ros_vec3.x = vn_vec3.c0;
  ros_vec3.y = vn_vec3.c1;
  ros_vec3.z = vn_vec3.c2;
}

void RosQuaternionFromVnQuaternion(geometry_msgs::Quaternion& ros_quat,
                                   const VnQuaternion& vn_quat) {
  ros_quat.x = vn_quat.x;
  ros_quat.y = vn_quat.y;
  ros_quat.z = vn_quat.z;
  ros_quat.w = vn_quat.w;
}

void FillImuMessage(sensor_msgs::Imu& imu_msg,
                    const VnDeviceCompositeData& data, bool binary_output) {
  if (binary_output) {
    RosQuaternionFromVnQuaternion(imu_msg.orientation, data.quaternion);
    // NOTE: The IMU angular velocity and linear acceleration outputs are
    // swapped. And also why are they different?
    
    RosVector3FromVnVector3(imu_msg.angular_velocity,
                            data.accelerationUncompensated);
    RosVector3FromVnVector3(imu_msg.linear_acceleration,
                            data.angularRateUncompensated);
  } 
  else {
    RosVector3FromVnVector3(imu_msg.linear_acceleration, data.acceleration);
    RosVector3FromVnVector3(imu_msg.angular_velocity, data.angularRate);
  }
}


}  //  namespace imu_vn_100
