#include "pico_zense_manager.hpp"

using namespace std;

PicoZenseManager::PicoZenseManager() {
  for (int deviceIndex = 0; deviceIndex < MAX_DEVICECOUNT; deviceIndex++) {
    deviceState_[deviceIndex] = DeviceClosed;
    serialNumber_[deviceIndex] = "";
    isWDR_[deviceIndex] = false;
    isRGB_[deviceIndex] = false;
  }

  PsReturnStatus status;

  status = PsInitialize();
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsInitialize failed!" << endl;
    exit(EXIT_FAILURE);
  }

  deviceCount_ = 0;
  status = PsGetDeviceCount(&deviceCount_);
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsGetDeviceCount failed!" << endl;
    exit(EXIT_FAILURE);
  }
  cout << "Detected " << deviceCount_ << " devices." << endl;
  if (deviceCount_ > MAX_DEVICECOUNT) {
    cout << "# of devices exceeds maximum of " << MAX_DEVICECOUNT << endl;
    deviceCount_ = MAX_DEVICECOUNT;
  }
}

PicoZenseManager::~PicoZenseManager() {
  PsReturnStatus status;
  status = PsShutdown();
  cout << "Shutdown : " << status << endl;
}

/*
int32_t PicoZenseManager::openDeviceBySerial(string strSerial, int32_t
_deviceIndex)
{
    int32_t ret = -1;
    for (int deviceIndex=0; deviceIndex<deviceCount_; deviceIndex++)
    {
        openDevice(deviceIndex);
        if (serialNumber_[deviceIndex] == strSerial)
        {
            ret = deviceIndex;
            break;
        }
        else closeDevice(deviceIndex);
    }
    return ret;
}
*/

// fix

void PicoZenseManager::openDeviceByIdx(int32_t _deviceIndex) {
  openDevice(_deviceIndex);
}

bool PicoZenseManager::openAllDevices() {
  for (int deviceIndex = 0; deviceIndex < deviceCount_; deviceIndex++) {
    if (!openDevice(deviceIndex)) return false;
  }
  return true;
}

bool PicoZenseManager::openDevice(int32_t deviceIndex) {
  cout << "Opening device : " << deviceIndex << endl;
  if (!(deviceIndex >= 0 && deviceIndex < deviceCount_)) {
    cout << "Device index is out of range!" << endl;
    return false;
  }

  if (deviceState_[deviceIndex] != DeviceClosed) {
    cout << "Device is already opened" << endl;
    return false;
  }

  PsReturnStatus status;

  status = PsOpenDevice(deviceIndex);
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsOpenDevice failed!" << endl;
    return false;
  }

  int32_t lenSerial = 100;
  char buffSerial[lenSerial];
  status = PsGetProperty(deviceIndex, PsPropertySN_Str, buffSerial, &lenSerial);
  serialNumber_[deviceIndex] = buffSerial;
  cout << "SERIAL : " << buffSerial << endl;

  int32_t lenHW = 100;
  char buffHW[lenHW];
  status = PsGetProperty(deviceIndex, PsPropertyHWVer_Str, buffHW, &lenHW);
  cout << "HW_VER : " << buffHW << endl;

  int32_t lenFW = 100;
  char buffFW[lenFW];
  status = PsGetProperty(deviceIndex, PsPropertyFWVer_Str, buffFW, &lenFW);
  cout << "FW_VER : " << buffFW << endl;

  cout << endl;

  deviceState_[deviceIndex] = DeviceOpened;
  return true;
}

void PicoZenseManager::closeAllDevices() {
  for (int deviceIndex = 0; deviceIndex < deviceCount_; deviceIndex++) {
    closeDevice(deviceIndex);
  }
}

void PicoZenseManager::closeDevice(int32_t deviceIndex) {
  if (deviceState_[deviceIndex] == DeviceClosed) return;

  PsReturnStatus status;

  /* Though it is recommended in SDK document, PsStopFrame call causes error and
  SDK sample programs are not using it so commenting out the below lines. if
  (deviceState_[deviceIndex] == DeviceStarted)
      {
          if (isWDR_[deviceIndex])
          {
              status = PsStopFrame(deviceIndex, PsWDRDepthFrame);
          }
          else
          {
              status = PsStopFrame(deviceIndex, PsDepthFrame);
              status = PsStopFrame(deviceIndex, PsIRFrame);
  //            status = PsStopFrame(deviceIndex, PsRGBFrame);
          }
      }
  */
  status = PsCloseDevice(deviceIndex);
  cout << "Closed device : " << deviceIndex << endl;
  deviceState_[deviceIndex] = DeviceClosed;
}

bool PicoZenseManager::setupDevice(int32_t deviceIndex, int32_t range1,
                                   int32_t range2, bool isRGB) {
  cout << "Setting up device : " << deviceIndex << endl;
  if (!(deviceIndex >= 0 && deviceIndex < deviceCount_)) {
    cout << "Device index is out of range!" << endl;
    return false;
  }

  if (deviceState_[deviceIndex] == DeviceClosed) return false;

  PsReturnStatus status;

  // Operating Mode
  int32_t dataMode;
  isWDR_[deviceIndex] = !(range2 < PsNearRange);
  isRGB_[deviceIndex] = isRGB;
  if (isWDR_[deviceIndex]) dataMode = PsWDR_Depth;
  //    else if (isRGB_[deviceIndex]) dataMode = PsDepthAndRGB_30;
  else if (isRGB_[deviceIndex])
    dataMode = PsDepthAndIR_15_RGB_30;
  else
    dataMode = PsDepthAndIR_30;
  status = PsSetDataMode(deviceIndex, (PsDataMode)dataMode);
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsSetDataMode failed!" << endl;
    return false;
  }

  // Set depth range
  string strRange[10];
  strRange[PsNearRange] = "near";
  strRange[PsMidRange] = "mid";
  strRange[PsFarRange] = "far";

  status = PsSetDepthRange(deviceIndex, (PsDepthRange)range1);
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsSetDepthRange failed!" << endl;
    return false;
  }
  if (isWDR_[deviceIndex]) {
    PsWDROutputMode modeWDR = {PsWDRTotalRange_Two,
                               (PsDepthRange)range1,
                               1,
                               (PsDepthRange)range2,
                               1,
                               PsNearRange,
                               1};
    status = PsSetWDROutputMode(deviceIndex, &modeWDR);
    if (status != PsReturnStatus::PsRetOK) {
      cout << "PsSetWDROutputMode failed!" << endl;
      return false;
    }
    status = PsSetWDRStyle(deviceIndex, PsWDR_ALTERNATION);
    if (status != PsReturnStatus::PsRetOK) {
      cout << "PsSetWDRStyle failed!" << endl;
      return false;
    }
    cout << "WDR mode : " << strRange[range1] << "-" << strRange[range2]
         << endl;
  } else {
    cout << "Single range mode : " << strRange[range1] << endl;
  }

  // Distortion
  PsSetDepthDistortionCorrectionEnabled(deviceIndex, true);
  PsSetIrDistortionCorrectionEnabled(deviceIndex, true);
  PsSetRGBDistortionCorrectionEnabled(deviceIndex, true);

  // TODO: tune these!!!
  // Filters
  PsSetFilter(deviceIndex, PsComputeRealDepthFilter, true);  // default : true
  PsSetFilter(deviceIndex, PsSmoothingFilter, true);         // default : true
  //    PsSetFilter(deviceIndex, PSSpatialFilter, false);

  // RGB resolution
  PsFrameMode frameMode;
  frameMode.fps = 30;
  frameMode.pixelFormat = PsPixelFormatBGR888;
  frameMode.resolutionWidth = 1920;
  frameMode.resolutionHeight = 1080;
  PsSetFrameMode(deviceIndex, PsRGBFrame, &frameMode);

  PsSetColorPixelFormat(deviceIndex, PsPixelFormatBGR888);

  status = PsSetMapperEnabledRGBToDepth(deviceIndex, false);
  if (status != PsRetOK) {
    cout << "PsSetMapperEnabledRGBToDepth failed!" << endl;
    return false;
  }

  status = PsSetMapperEnabledDepthToRGB(deviceIndex, false);
  if (status != PsRetOK) {
    cout << "PsSetMapperEnabledDepthToRGB failed!" << endl;
    return false;
  }

  //////////////////////////////////////////////////////////////////////////////
  /* TODO memos!!!!!

  PsDepthVector3

  PsSetWDRFusionThreshold

  PsGetThreshold
  PsSetThreshold

  PsGetTimeFilterEnabled
  PsSetTimeFilterEnabled

  PsSetComputeRealDepthCorrectionEnabled
  PsSetSmoothingFilterEnabled
  PsSetSpatialFilterEnabled

  */
  //////////////////////////////////////////////////////////////////////////////

  cameraParams_[deviceIndex][0] =
      updateCameraParameter_(deviceIndex, PsDepthSensor);
  cameraParams_[deviceIndex][1] =
      updateCameraParameter_(deviceIndex, PsRgbSensor);

  return true;
}

bool PicoZenseManager::startDevice(int32_t deviceIndex) {
  if (deviceState_[deviceIndex] == DeviceClosed) return false;
  if (deviceState_[deviceIndex] == DeviceStarted) return false;

  PsReturnStatus status;
  /* Though it is recommended in SDK document, PsStopFrame call causes error and
  SDK sample programs are not using it so commenting out the below lines. if
  (isWDR_[deviceIndex])
      {
          status = PsStartFrame(deviceIndex, PsWDRDepthFrame);
      }
      else
      {
          status = PsStartFrame(deviceIndex, PsDepthFrame);
          status = PsStartFrame(deviceIndex, PsIRFrame);
  //        status = PsStartFrame(deviceIndex, PsRGBFrame);
      }
      if (status != PsRetOK)
      {
          cout << "PsStartFrame failed!" << endl;
          return false;
      }
  */
  cout << "Started capturing on device : " << deviceIndex << endl;
  cout << endl;

  deviceState_[deviceIndex] = DeviceStarted;
  return true;
}

bool PicoZenseManager::updateDevice(int32_t deviceIndex) {
  bool isSuccess = false;
  PsReturnStatus status;

  if (deviceState_[deviceIndex] != DeviceStarted) {
    cout << "Device " << deviceIndex << " has not started!" << endl;
    return isSuccess;
  }
  status = PsReadNextFrame(deviceIndex);

  if (status != PsRetOK) {
    cout << "Could not read next frame from device " << deviceIndex << endl;
    return isSuccess;
  }
  //imageTimestamps_[deviceIndex] = ros::Time::now();

  if (isWDR_[deviceIndex]) {
    // WDR Depth
    PsFrame depthFrame = {0};
    PsGetFrame(deviceIndex, PsWDRDepthFrame, &depthFrame);
    if (depthFrame.pFrameData != NULL) {
      depthRange_[deviceIndex] = depthFrame.depthRange;
      depthImg_[deviceIndex] = cv::Mat(depthFrame.height, depthFrame.width,
                                       CV_16UC1, depthFrame.pFrameData);
      irImg_[deviceIndex] = depthImg_[deviceIndex];
      isSuccess = true;
    }
  } else {
    // Depth
    PsFrame depthFrame = {0};
    PsGetFrame(deviceIndex, PsDepthFrame, &depthFrame);
    if (depthFrame.pFrameData != NULL) {
      depthRange_[deviceIndex] = depthFrame.depthRange;
      depthImg_[deviceIndex] = cv::Mat(depthFrame.height, depthFrame.width,
                                       CV_16UC1, depthFrame.pFrameData);
      irImg_[deviceIndex] = depthImg_[deviceIndex];
      isSuccess = true;
    }

    // IR
    PsFrame irFrame = {0};
    PsGetFrame(deviceIndex, PsIRFrame, &irFrame);
    if (irFrame.pFrameData != NULL) {
      irImg_[deviceIndex] =
          cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
    }

    // RGB
    if (isRGB_[deviceIndex]) {
      PsFrame rgbFrame = {0};
      PsGetFrame(deviceIndex, PsRGBFrame, &rgbFrame);
      if (rgbFrame.pFrameData != NULL) {
        rgbImg_[deviceIndex] = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3,
                                       rgbFrame.pFrameData);
      }
    }
  }

  if (isSuccess) {
    if (depthImg_[deviceIndex].rows == 0) isSuccess = false;
    if (irImg_[deviceIndex].rows == 0) isSuccess = false;
  }

  return isSuccess;
}

int32_t PicoZenseManager::getDeviceIndex(string strSerial) {
  int32_t ret = -1;
  for (int deviceIndex = 0; deviceIndex < deviceCount_; deviceIndex++) {
    if (serialNumber_[deviceIndex] == strSerial) {
      ret = deviceIndex;
      break;
    }
  }
  if (ret < 0) {
    cout << "Device with serial# " << strSerial << " is not found!" << endl;
  } else {
    cout << "Device with serial# " << strSerial << " is found at index :" << ret
         << endl;
  }
  return ret;
}

CameraParameter PicoZenseManager::updateCameraParameter_(
    int32_t deviceIndex, PsSensorType sensor_type) {
  PsReturnStatus status;
  PsCameraParameters cameraParameters;

  std::string sensor_type_str;

  status = PsGetCameraParameters(deviceIndex, sensor_type, &cameraParameters);

  CameraParameter cameraParam;
  //    cameraParam.image_width = cameraParameters.;
  //    cameraParam.image_height = cameraParameters.;
  cameraParam.fx = cameraParameters.fx;
  cameraParam.fy = cameraParameters.fy;
  cameraParam.cx = cameraParameters.cx;
  cameraParam.cy = cameraParameters.cy;
  cameraParam.p1 = cameraParameters.p1;
  cameraParam.p2 = cameraParameters.p2;
  cameraParam.k1 = cameraParameters.k1;
  cameraParam.k2 = cameraParameters.k2;
  cameraParam.k3 = cameraParameters.k3;
  cameraParam.k4 = cameraParameters.k4;
  cameraParam.k5 = cameraParameters.k5;
  cameraParam.k6 = cameraParameters.k6;

  if (sensor_type == PsDepthSensor) {
    sensor_type_str = "Depth camera";
  } else {
    sensor_type_str = "RGB camera";
  }

  cout << "Intinsic parameters of device : " << deviceIndex << endl;
  cout << "serial_no = \"" << serialNumber_[deviceIndex] << "\"" << endl;
  cout << "Camera type: " << sensor_type_str << endl;
  printCameraParams(cameraParam);

  return cameraParam;
}
