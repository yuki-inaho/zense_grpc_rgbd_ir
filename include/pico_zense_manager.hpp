#pragma once
#include "PicoZense_api.h"
#include "common.hpp"

#define MAX_DEVICECOUNT 10

class PicoZenseManager {
 public:
  PicoZenseManager();
  ~PicoZenseManager();

  void openDeviceByIdx(int32_t _deviceIdx);

  bool openAllDevices();
  bool openDevice(int32_t deviceIndex);

  void closeAllDevices();
  void closeDevice(int32_t deviceIndex);

  bool setupDevice(int32_t deviceIndex, int32_t range1 = PsNearRange,
                   int32_t range2 = PsFarRange, bool isRGB = false);
  bool startDevice(int32_t deviceIndex);

  bool updateDevice(int32_t deviceIndex);

  int32_t getDeviceCount() { return deviceCount_; }
  std::string getSerialNumber(int32_t deviceIndex) {
    return serialNumber_[deviceIndex];
  }
  int32_t getDeviceIndex(std::string strSerial);

  void setSmoothingFilter(int32_t deviceIndex, bool enable) {
    PsSetFilter(deviceIndex, PsSmoothingFilter, enable);
    std::cout << "Device " << deviceIndex
              << " SmoothingFilter : " << (enable ? "ON" : "OFF") << std::endl;
  }

  bool isWDR(int32_t deviceIndex) { return isWDR_[deviceIndex]; }
  bool isRGB(int32_t deviceIndex) { return isRGB_[deviceIndex]; }

  /*
  ros::Time getImageTimestamp(int32_t deviceIndex) {
    return imageTimestamps_[deviceIndex];
  }
  */
  int32_t getDepthRange(int32_t deviceIndex) {
    return depthRange_[deviceIndex];
  }

  cv::Mat getDepthImage(int32_t deviceIndex) { return depthImg_[deviceIndex]; }
  cv::Mat getIRImage(int32_t deviceIndex) { return irImg_[deviceIndex]; }
  cv::Mat getRgbImage(int32_t deviceIndex) { return rgbImg_[deviceIndex]; }

  // @param sensor_type 0: depth, 1: rgb
  CameraParameter getCameraParameter(int32_t deviceIndex, int32_t sensor_type) {
    return cameraParams_[deviceIndex][sensor_type];
  }

  typedef enum {
    DeviceClosed = 0,
    DeviceOpened = 1,
    DeviceStarted = 2,
  } DeviceState;

 private:
  int32_t deviceCount_;

  DeviceState deviceState_[MAX_DEVICECOUNT];
  std::string serialNumber_[MAX_DEVICECOUNT];
  bool isWDR_[MAX_DEVICECOUNT];
  bool isRGB_[MAX_DEVICECOUNT];

  //ros::Time imageTimestamps_[MAX_DEVICECOUNT];
  int32_t depthRange_[MAX_DEVICECOUNT];
  cv::Mat depthImg_[MAX_DEVICECOUNT];
  cv::Mat irImg_[MAX_DEVICECOUNT];
  cv::Mat rgbImg_[MAX_DEVICECOUNT];

  CameraParameter cameraParams_[MAX_DEVICECOUNT][2];  // 0: depth, 1: rgb

  CameraParameter updateCameraParameter_(
      int32_t deviceIndex, PsSensorType sensor_type = PsDepthSensor);
};
