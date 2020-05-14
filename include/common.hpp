#pragma once

#include <omp.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

typedef struct{
  int image_width;
  int image_height;
  float fx;  //!< Focal length x (pixel)
  float fy;  //!< Focal length y (pixel)
  float cx;  //!< Principal point x (pixel)
  float cy;  //!< Principal point y (pixel)
  float p1;  //!< Tangential distortion coefficient
  float p2;  //!< Tangential distortion coefficient
  float k1;  //!< Radial distortion coefficient, 1st-order
  float k2;  //!< Radial distortion coefficient, 2nd-order
  float k3;  //!< Radial distortion coefficient, 3rd-order
  float k4;  //!< Radial distortion coefficient, 4st-order
  float k5;  //!< Radial distortion coefficient, 5nd-order
  float k6;  //!< Radial distortion coefficient, 6rd-order
}CameraParameter;

bool checkFileExistence(const std::string& str);
void printCameraParams(CameraParameter cameraParam);
cv::Mat cvtDepth2ColorScale(cv::Mat depthImg);
