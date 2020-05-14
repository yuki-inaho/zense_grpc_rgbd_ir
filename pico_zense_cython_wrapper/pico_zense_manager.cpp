#include "pico_zense_manager.hpp"

using namespace std;
namespace zense {
PicoZenseManager::PicoZenseManager(int32_t device_idx) {
  device_idx_ = device_idx;
  PsReturnStatus status;
  status = PsInitialize();
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsInitialize failed!" << endl;
    exit(EXIT_FAILURE);
  }

  int32_t deviceCount_ = 0;
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

  cout << "sensor_idx_:" << device_idx_ << endl;
  status = PsOpenDevice(device_idx_);
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsOpenDevice failed!" << endl;
    exit(EXIT_FAILURE);
  }

  // get Serial Number
  int32_t lenSerial = 100;
  char buffSerial[lenSerial];
  status = PsGetProperty(device_idx_, PsPropertySN_Str, buffSerial, &lenSerial);
  serialNumber_ = buffSerial;
  cout << "SERIAL : " << buffSerial << endl;

  // set Depth Camera Parameter
  PsCameraParameters camera_parameters;
  status =
      PsGetCameraParameters(device_idx_, PsDepthSensor, &camera_parameters);

  // status = PsSetDataMode(device_idx_, PsDepthAndIR_30);
  status = PsSetDataMode(device_idx_, PsDepthAndIR_15_RGB_30);
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsSetDataMode failed!" << endl;
    ;
  }

  status = PsSetResolution(device_idx_, PsRGB_Resolution_1920_1080);
  status = PsSetDepthRange(device_idx_, PsNearRange);
  if (status != PsReturnStatus::PsRetOK) {
    cout << "PsSetDepthRange failed!" << endl;
    exit(EXIT_FAILURE);
  }

  cout << "serial_no = \"" << serialNumber_ << "\"" << endl;
  camera_param_.fx = camera_parameters.fx;
  camera_param_.fy = camera_parameters.fy;
  camera_param_.cx = camera_parameters.cx;
  camera_param_.cy = camera_parameters.cy;
  camera_param_.p1 = camera_parameters.p1;
  camera_param_.p2 = camera_parameters.p2;
  camera_param_.k1 = camera_parameters.k1;
  camera_param_.k2 = camera_parameters.k2;
  camera_param_.k3 = camera_parameters.k3;
  camera_param_.k4 = camera_parameters.k4;
  camera_param_.k5 = camera_parameters.k5;
  camera_param_.k6 = camera_parameters.k6;

  PsCameraParameters camera_parameters_rgb;
  status =
      PsGetCameraParameters(device_idx_, PsRgbSensor, &camera_parameters_rgb);
  camera_param_rgb_.fx = camera_parameters_rgb.fx;
  camera_param_rgb_.fy = camera_parameters_rgb.fy;
  camera_param_rgb_.cx = camera_parameters_rgb.cx;
  camera_param_rgb_.cy = camera_parameters_rgb.cy;
  camera_param_rgb_.p1 = camera_parameters_rgb.p1;
  camera_param_rgb_.p2 = camera_parameters_rgb.p2;
  camera_param_rgb_.k1 = camera_parameters_rgb.k1;
  camera_param_rgb_.k2 = camera_parameters_rgb.k2;
  camera_param_rgb_.k3 = camera_parameters_rgb.k3;
  camera_param_rgb_.k4 = camera_parameters_rgb.k4;
  camera_param_rgb_.k5 = camera_parameters_rgb.k5;
  camera_param_rgb_.k6 = camera_parameters_rgb.k6;

  PsCameraExtrinsicParameters pCameraExtrinsicParameters;
  PsGetCameraExtrinsicParameters(device_idx_, &pCameraExtrinsicParameters);
  std::vector<double> _rotation(std::begin(pCameraExtrinsicParameters.rotation),
                                std::end(pCameraExtrinsicParameters.rotation));
  std::vector<double> _translation(
      std::begin(pCameraExtrinsicParameters.translation),
      std::end(pCameraExtrinsicParameters.translation));
  extrinsic_param_.rotation = _rotation;
  extrinsic_param_.translation = _translation;

  PsSetRGBDistortionCorrectionEnabled(device_idx_, true);
  PsSetDepthDistortionCorrectionEnabled(device_idx_, true);
  PsSetIrDistortionCorrectionEnabled(device_idx_, true);

  PsSetFilter(device_idx_, PsComputeRealDepthFilter, true);  // default : true
  PsSetFilter(device_idx_, PsSmoothingFilter, true);         // default : true

  PsSetSpatialFilterEnabled(device_idx_, true);  // default : true
  PsSetTimeFilterEnabled(device_idx_, true);     // default : true

  status = PsSetMapperEnabledRGBToDepth(device_idx_, false);
  if (status != PsRetOK) {
    cout << "PsSetMapperEnabledRGBToDepth failed!" << endl;
    exit(EXIT_FAILURE);
  }
  status = PsSetMapperEnabledDepthToRGB(device_idx_, false);
  if (status != PsRetOK) {
    cout << "PsSetMapperEnabledRGBToDepth failed!" << endl;
    exit(EXIT_FAILURE);
  }
}

PicoZenseManager::~PicoZenseManager() {
  PsReturnStatus status;
  status = PsShutdown();
  cout << "Shutdown : " << status << endl;
}

bool PicoZenseManager::update() {
  bool isSuccess = false;
  PsReturnStatus status;
  status = PsReadNextFrame(device_idx_);

  if (status != PsRetOK) {
    cout << "Could not read next frame from device " << device_idx_ << endl;
    return isSuccess;
  }

  // Depth
  PsFrame depthFrame = {0};
  PsGetFrame(device_idx_, PsDepthFrame, &depthFrame);
  if (depthFrame.pFrameData != NULL) {
    depthImg_ = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1,
                        depthFrame.pFrameData);
    isSuccess = true;
  }

  // IR
  PsFrame irFrame = {0};
  PsGetFrame(device_idx_, PsIRFrame, &irFrame);
  if (irFrame.pFrameData != NULL) {
    irImg_ =
        cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
    isSuccess = true;
  }

  PsFrame rgbFrame = {0};
  PsGetFrame(device_idx_, PsRGBFrame, &rgbFrame);
  if (rgbFrame.pFrameData != NULL) {
    rgbImg_ =
        cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);
    isSuccess = true;
  }

  if (isSuccess) {
    if (depthImg_.rows == 0) isSuccess = false;
    if (irImg_.rows == 0) isSuccess = false;
    if (rgbImg_.rows == 0) isSuccess = false;
  }

  return isSuccess;
}

void PicoZenseManager::printCameraParams(CameraParameter camera_param) {
  std::cout << "Camera parameters(Depth):" << std::endl;
  std::cout << "fx = " << camera_param.fx << std::endl;
  std::cout << "fy = " << camera_param.fy << std::endl;
  std::cout << "cx = " << camera_param.cx << std::endl;
  std::cout << "cy = " << camera_param.cy << std::endl;
  std::cout << "p1 = " << camera_param.p1 << std::endl;
  std::cout << "p2 = " << camera_param.p2 << std::endl;
  std::cout << "k1 = " << camera_param.k1 << std::endl;
  std::cout << "k2 = " << camera_param.k2 << std::endl;
  std::cout << "k3 = " << camera_param.k3 << std::endl;
  std::cout << "k4 = " << camera_param.k4 << std::endl;
  std::cout << "k5 = " << camera_param.k5 << std::endl;
  std::cout << "k6 = " << camera_param.k6 << std::endl;
}

std::vector<std::vector<double>> PicoZenseManager::getExtrinsicParameter() {
  std::vector<std::vector<double>> extrinsic_parameter_vec;
  extrinsic_parameter_vec.push_back(extrinsic_param_.rotation);
  extrinsic_parameter_vec.push_back(extrinsic_param_.translation);
  return extrinsic_parameter_vec;
}

std::vector<double> PicoZenseManager::getCameraParameter() {
  std::vector<double> camera_parameter_vec;
  camera_parameter_vec.push_back(camera_param_.fx);
  camera_parameter_vec.push_back(camera_param_.fy);
  camera_parameter_vec.push_back(camera_param_.cx);
  camera_parameter_vec.push_back(camera_param_.cy);
  camera_parameter_vec.push_back(camera_param_.p1);
  camera_parameter_vec.push_back(camera_param_.p2);
  camera_parameter_vec.push_back(camera_param_.k1);
  camera_parameter_vec.push_back(camera_param_.k2);
  camera_parameter_vec.push_back(camera_param_.k3);
  camera_parameter_vec.push_back(camera_param_.k4);
  camera_parameter_vec.push_back(camera_param_.k5);
  camera_parameter_vec.push_back(camera_param_.k6);
  return camera_parameter_vec;
}

std::vector<double> PicoZenseManager::getRGBCameraParameter() {
  std::vector<double> camera_parameter_vec;
  camera_parameter_vec.push_back(camera_param_rgb_.fx);
  camera_parameter_vec.push_back(camera_param_rgb_.fy);
  camera_parameter_vec.push_back(camera_param_rgb_.cx);
  camera_parameter_vec.push_back(camera_param_rgb_.cy);
  camera_parameter_vec.push_back(camera_param_rgb_.p1);
  camera_parameter_vec.push_back(camera_param_rgb_.p2);
  camera_parameter_vec.push_back(camera_param_rgb_.k1);
  camera_parameter_vec.push_back(camera_param_rgb_.k2);
  camera_parameter_vec.push_back(camera_param_rgb_.k3);
  camera_parameter_vec.push_back(camera_param_rgb_.k4);
  camera_parameter_vec.push_back(camera_param_rgb_.k5);
  camera_parameter_vec.push_back(camera_param_rgb_.k6);
  return camera_parameter_vec;
}

std::string PicoZenseManager::getSerialNumber() { return serialNumber_; }

cv::Mat PicoZenseManager::getIRImage() { return irImg_; }

cv::Mat PicoZenseManager::getDepthImage() { return depthImg_; }

cv::Mat PicoZenseManager::getRGBImage() { return rgbImg_; }

}  // namespace zense
