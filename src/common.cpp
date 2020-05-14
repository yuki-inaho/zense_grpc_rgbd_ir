#include "common.hpp"

bool checkFileExistence(const std::string &str)
{
  std::ifstream ifs(str);
  return ifs.is_open();
}

void printCameraParams(CameraParameter cameraParam)
{
  std::cout << "Camera parameters:" << std::endl;

  //    std::cout << "image_width = " << cameraParam.image_width << std::endl;
  //    std::cout << "image_height = " << cameraParam.image_height << std::endl;

  std::cout << "fx = " << cameraParam.fx << std::endl;
  std::cout << "fy = " << cameraParam.fy << std::endl;
  std::cout << "cx = " << cameraParam.cx << std::endl;
  std::cout << "cy = " << cameraParam.cy << std::endl;

  std::cout << "p1 = " << cameraParam.p1 << std::endl;
  std::cout << "p2 = " << cameraParam.p2 << std::endl;
  std::cout << "k1 = " << cameraParam.k1 << std::endl;
  std::cout << "k2 = " << cameraParam.k2 << std::endl;
  std::cout << "k3 = " << cameraParam.k3 << std::endl;
  std::cout << "k4 = " << cameraParam.k4 << std::endl;
  std::cout << "k5 = " << cameraParam.k5 << std::endl;
  std::cout << "k6 = " << cameraParam.k6 << std::endl;
}

cv::Mat cvtDepth2ColorScale(cv::Mat depthImg)
{
  cv::Mat tmp, tmpColor;
  depthImg.convertTo(tmp, CV_8U, 255.0 * 0.001 / 1.2);
  cv::applyColorMap(tmp, tmpColor, cv::COLORMAP_JET);
  return tmpColor;
}
