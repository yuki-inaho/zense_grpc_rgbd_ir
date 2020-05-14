#include "parameter_manager.hpp"
#include "pico_zense_manager.hpp"
#include "pico_zense_undistorter.hpp"

#include "common.hpp"

#include <unordered_map>
#include <grpcpp/grpcpp.h>
#include "unistd.h" 
#include <chrono>
#include "image.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using image::ImageRequest;
using image::ImageRGBDReply;
using image::ImageService;
using image::ImageWDRReply;

#define INIT_SKIP_COUNTER -200
#define MAX_SKIP_COUNTER 30
#define MAX_HEARTBEAT_COUNTER 10

class PicoZenseSimplePublisher final : public ImageService::Service
{
private:
  // PicoZense custom API
  PicoZenseManager manager_;

  int range1;
  int range2;

  int skip_counter_[3] = {INIT_SKIP_COUNTER, INIT_SKIP_COUNTER,
                          INIT_SKIP_COUNTER};
  bool flag_wdr_range_updated_[3] = {false, false, false}; // assumed near = 0, mid = 1, far = 2

  int device_index_;
  std::string sensor_id_;
  std::string serial_no_;

  bool isRGB, isWDR;

  cv::Mat rgb_image;
  cv::Mat depth_image_range1, depth_image_range2;

  bool undistortion_flag;
  PicoZenseUndistorter undistorter;

  CameraParameter camera_param;

public:
  PicoZenseSimplePublisher(std::string cfgParamPath, std::string camKey, int device_index__);
  ~PicoZenseSimplePublisher();
  bool isWithinError(float val, float ref);
  bool isWDRUpdated();
  void close();
  bool update();
  uint64_t getTimeStamp();
  void SetRGBImageReply(ImageRGBDReply *reply);
  void SetDepthImageReply(ImageRGBDReply *reply);
  void _setWDRDepthImageReply(ImageWDRReply *reply, int &width, int &height,
                              int &channel, int &image_size,
                              std::string &data_depth_str, bool is_first_range);
  void SetWDRDepthImageReply(ImageWDRReply *reply);
  Status SendRGBDImage(ServerContext *context, const ImageRequest *request,
                       ImageRGBDReply *reply) override;
  Status SendWDRImage(ServerContext *context, const ImageRequest *request,
                      ImageWDRReply *reply) override;
};

PicoZenseSimplePublisher::PicoZenseSimplePublisher(std::string cfgParamPath,
                                                   std::string camKey,
                                                   int device_index__)
{
  device_index_ = device_index__;
  usleep(5 * 1e6); // To avoid high frequent sensor open call from immediately
                   // after termination and rebooting

  if (!checkFileExistence(cfgParamPath))
  {
    std::cerr << "Setting TOML file does not exist" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  ParameterManager cfgParam(cfgParamPath);

  sensor_id_ = cfgParam.readStringData("General", "sensor_id");

  if (!cfgParam.checkExistanceTable(camKey.c_str()))
  {
    std::cerr << "Camera name is invalid, please check setting toml file" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::string camera_name =
      cfgParam.readStringData(camKey.c_str(), "camera_name");
  serial_no_ = cfgParam.readStringData(camKey.c_str(), "serial_no");
  range1 = cfgParam.readIntData(camKey.c_str(), "range1");
  range2 = cfgParam.readIntData(camKey.c_str(), "range2");
  isRGB = cfgParam.readIntData(camKey.c_str(), "rgb_image") == 1;
  isWDR = (range1 >= 0) && (range2 >= 0);

  // TODO: merge factory values and tuned values
  std::string camFactKey = camKey + "_Factory";

  CameraParameter camera_factory_param;
  camera_factory_param.fx = cfgParam.readFloatData(camFactKey.c_str(), "fx");
  camera_factory_param.fy = cfgParam.readFloatData(camFactKey.c_str(), "fy");
  camera_factory_param.cx = cfgParam.readFloatData(camFactKey.c_str(), "cx");
  camera_factory_param.cy = cfgParam.readFloatData(camFactKey.c_str(), "cy");
  camera_factory_param.p1 = cfgParam.readFloatData(camFactKey.c_str(), "p1");
  camera_factory_param.p2 = cfgParam.readFloatData(camFactKey.c_str(), "p2");
  camera_factory_param.k1 = cfgParam.readFloatData(camFactKey.c_str(), "k1");
  camera_factory_param.k2 = cfgParam.readFloatData(camFactKey.c_str(), "k2");
  camera_factory_param.k3 = cfgParam.readFloatData(camFactKey.c_str(), "k3");
  camera_factory_param.k4 = cfgParam.readFloatData(camFactKey.c_str(), "k4");
  camera_factory_param.k5 = cfgParam.readFloatData(camFactKey.c_str(), "k5");
  camera_factory_param.k6 = cfgParam.readFloatData(camFactKey.c_str(), "k6");

  std::string id_str;
  std::cout << "Serial number allocated to Zense Publisher : " << serial_no_ << std::endl;

  /* Define Undistortion Module */
  std::string distortionKey = camKey + "_Undistortion";

  // If TOML configuration file doesn't contain any undistortion
  // description(table), undistortion process will be skip
  if (cfgParam.checkExistanceTable(distortionKey)) {
    undistortion_flag = cfgParam.readBoolData(distortionKey, "flag");
  } else {
    undistortion_flag = false;
  }
  if (undistortion_flag == true) {
    undistorter = PicoZenseUndistorter(cfgParam, distortionKey);
  }

  manager_.openDevice(device_index_);
  if (!manager_.setupDevice(device_index_, range1, range2, isRGB))
  {
    close();
    std::cerr << "Could not setup device" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  if (range2 < 0)
    range2 = range1;

  camera_param = manager_.getCameraParameter(device_index_, 0);
  if (!(isWithinError(camera_param.fx, camera_factory_param.fx) &&
        isWithinError(camera_param.fy, camera_factory_param.fy) &&
        isWithinError(camera_param.cx, camera_factory_param.cx) &&
        isWithinError(camera_param.cy, camera_factory_param.cy) &&
        isWithinError(camera_param.p1, camera_factory_param.p1) &&
        isWithinError(camera_param.p2, camera_factory_param.p2) &&
        isWithinError(camera_param.k1, camera_factory_param.k1) &&
        isWithinError(camera_param.k2, camera_factory_param.k2) &&
        isWithinError(camera_param.k3, camera_factory_param.k3) &&
        isWithinError(camera_param.k4, camera_factory_param.k4) &&
        isWithinError(camera_param.k5, camera_factory_param.k5) &&
        isWithinError(camera_param.k6, camera_factory_param.k6)))
  {
    close();
    std::cerr << "Erroneous internal parameters. Exiting..." << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (!manager_.startDevice(device_index_))
  {
    close();
    std::cerr << "Could not start device" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::string ns = "/" + camera_name;

  std::cout << "Camera setup is finished!" << std::endl;
}

PicoZenseSimplePublisher::~PicoZenseSimplePublisher() { close(); }

bool PicoZenseSimplePublisher::isWithinError(float val, float ref)
{
  const double fract_err = 1e-5;
  return (std::fabs(val - ref) <= fract_err * std::fabs(ref));
}

void PicoZenseSimplePublisher::close()
{
  manager_.closeDevice(device_index_);
}

bool PicoZenseSimplePublisher::update()
{
  skip_counter_[range1]++;
  skip_counter_[range2]++;
  if (skip_counter_[range1] > MAX_SKIP_COUNTER ||
      skip_counter_[range2] > MAX_SKIP_COUNTER)
  {
    close();
    std::cerr << "Device not responding. Exiting..." << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (!manager_.updateDevice(device_index_))
  {
    std::cout << "Device not updated. Skipping..." << std::endl;
    usleep(33333);
    return false;
  }

  int32_t depth_range = manager_.getDepthRange(device_index_);
  if (isWDR)
  {
    cv::Mat _depth_image = manager_.getDepthImage(device_index_);    
    if (depth_range == range1)
    {
        depth_image_range1 = _depth_image.clone();
        if (undistortion_flag){
          undistorter.undistortion(depth_image_range1, camera_param, range1);
        }
    }
    else if (depth_range == range2)
    {
        depth_image_range2 = _depth_image.clone();
        if (undistortion_flag){
          undistorter.undistortion(depth_image_range2, camera_param, range2);
        }
    }
    else
    {
      std::cerr << "Aquired depth range is invalid" << std::endl;
      close();
      std::exit(EXIT_FAILURE);
    }
  }
  skip_counter_[depth_range] = 0;
  flag_wdr_range_updated_[depth_range] = true;
  return true;
}

uint64_t PicoZenseSimplePublisher::getTimeStamp(){
  uint64_t microsecondsUTC = std::chrono::duration_cast<std::chrono::microseconds>
                            (std::chrono::system_clock::now().time_since_epoch()).count();
  return microsecondsUTC;
}

void PicoZenseSimplePublisher::SetRGBImageReply(ImageRGBDReply *reply)
{
  cv::Mat _rgb_image;
  _rgb_image = manager_.getRgbImage(device_index_);
  int width = _rgb_image.cols;
  int height = _rgb_image.rows;
  int channel = _rgb_image.channels();

  int image_size = _rgb_image.total() * _rgb_image.elemSize();
  char *image_char = new char[image_size];
  std::memcpy(image_char, _rgb_image.data, image_size * sizeof(char));
  std::string data_rgb_str(reinterpret_cast<char const *>(image_char), image_size);

  image::Image *image_rgb_ptr = reply->mutable_image_rgb();
  image_rgb_ptr->set_width(width);
  image_rgb_ptr->set_height(height);
  image_rgb_ptr->set_channel(channel);
  image_rgb_ptr->set_image_size(image_size);
  image_rgb_ptr->set_data(data_rgb_str);
  image_rgb_ptr->set_timestamp(getTimeStamp());
}

void PicoZenseSimplePublisher::SetDepthImageReply(ImageRGBDReply *reply)
{
  cv::Mat _depth_image = manager_.getDepthImage(device_index_);
  int32_t _depth_range = manager_.getDepthRange(device_index_);
  
  if (undistortion_flag){
    undistorter.undistortion(_depth_image, camera_param, _depth_range);
  }

  int width = _depth_image.cols;
  int height = _depth_image.rows;
  int channel = _depth_image.channels();

  int image_size = _depth_image.total() * _depth_image.elemSize();
  char *image_char = new char[image_size];
  std::memcpy(image_char, _depth_image.data, image_size * sizeof(char));
  std::string data_depth_str(reinterpret_cast<char const *>(image_char), image_size);

  image::Image *image_depth_ptr = reply->mutable_image_depth();
  image_depth_ptr->set_width(width);
  image_depth_ptr->set_height(height);
  image_depth_ptr->set_channel(channel);
  image_depth_ptr->set_image_size(image_size);
  image_depth_ptr->set_depth_range(manager_.getDepthRange(device_index_));
  image_depth_ptr->set_data(data_depth_str);
  image_depth_ptr->set_timestamp(getTimeStamp());
}

void PicoZenseSimplePublisher::
    _setWDRDepthImageReply(ImageWDRReply *reply, int &width, int &height,
                           int &channel, int &image_size,
                           std::string &data_depth_str, bool is_first_range)
{
  char* image_char;
  if (is_first_range)
  {
    width = depth_image_range1.cols;
    height = depth_image_range1.rows;
    channel = depth_image_range1.channels();
    image_size = depth_image_range1.total() * depth_image_range1.elemSize();
    image_char = new char[image_size];
    std::memcpy(image_char, depth_image_range1.data, image_size * sizeof(char));
  }
  else
  {
    width = depth_image_range2.cols;
    height = depth_image_range2.rows;
    channel = depth_image_range2.channels();
    image_size = depth_image_range2.total() * depth_image_range2.elemSize();
    image_char = new char[image_size];
    std::memcpy(image_char, depth_image_range2.data, image_size * sizeof(char));
  }
  std::string _data_depth_str(reinterpret_cast<char const *>(image_char), image_size);
  data_depth_str = _data_depth_str;

  if (is_first_range)
  {
    image::Image *image_depth_ptr = reply->mutable_image_depth_range1();
    image_depth_ptr->set_width(width);
    image_depth_ptr->set_height(height);
    image_depth_ptr->set_channel(channel);
    image_depth_ptr->set_image_size(image_size);
    image_depth_ptr->set_depth_range(range1);
    image_depth_ptr->set_data(data_depth_str);
    image_depth_ptr->set_timestamp(getTimeStamp());
  }
  else
  {
    image::Image *image_depth_ptr = reply->mutable_image_depth_range2();
    image_depth_ptr->set_width(width);
    image_depth_ptr->set_height(height);
    image_depth_ptr->set_channel(channel);
    image_depth_ptr->set_image_size(image_size);
    image_depth_ptr->set_depth_range(range2);
    image_depth_ptr->set_data(data_depth_str);
    image_depth_ptr->set_timestamp(getTimeStamp());
  }
}

void PicoZenseSimplePublisher::SetWDRDepthImageReply(ImageWDRReply *reply)
{
  int width_r1, height_r1, channel_r1, image_size_r1;
  std::string data_depth_str_r1;
  _setWDRDepthImageReply(reply, width_r1, height_r1, channel_r1, image_size_r1, data_depth_str_r1, true);
  int width_r2, height_r2, channel_r2, image_size_r2;
  std::string data_depth_str_r2;
  _setWDRDepthImageReply(reply, width_r2, height_r2, channel_r2, image_size_r2, data_depth_str_r2, false);
}

bool PicoZenseSimplePublisher::isWDRUpdated()
{
  return flag_wdr_range_updated_[range1] && flag_wdr_range_updated_[range2];
}

Status PicoZenseSimplePublisher::SendWDRImage(ServerContext *context, const ImageRequest *request,
                                              ImageWDRReply *reply)
{
  // If skip_counter_[*] exceeds MAX_SKIP_COUNTER in update(),
  // the sensor module will terminate itself. So, infinite loop can be avoided in this process.
  int wdr_skip_counter = 0;
  while (!update() || !isWDRUpdated())
  {
    if (wdr_skip_counter > MAX_SKIP_COUNTER)
    {
      close();
      std::cerr << "WDR Depth infomation couldn't bo correctly aquired. Exiting..." << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }
  SetWDRDepthImageReply(reply);
  flag_wdr_range_updated_[range1] = false;
  flag_wdr_range_updated_[range2] = false;
  return Status::OK;
}

Status PicoZenseSimplePublisher::SendRGBDImage(ServerContext *context, const ImageRequest *request,
                                               ImageRGBDReply *reply)
{

  // If skip_counter_[*] exceeds MAX_SKIP_COUNTER in update(),
  // the sensor module will terminate itself. So, infinite loop can be avoided in this process.
  while (!update())
    ;
  SetRGBImageReply(reply);
  SetDepthImageReply(reply);
  return Status::OK;
}

int main(int argc, char **argv)
{
  int32_t device_index = 0;
  if (argc >= 2)
  {
    device_index = std::stoi(argv[1]);
  }

  std::string cfgParamPath = "../cfg/camera.toml";
  if (argc >= 3)
  {
    cfgParamPath = argv[2];
  }

  std::string camKey = "Camera0";
  if (argc >= 4)
  {
    camKey = argv[3];
  }

  std::string server_address("localhost:50051");
  PicoZenseSimplePublisher pub(cfgParamPath, camKey, device_index);

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&pub);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
  server->Wait();
}