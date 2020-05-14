#include "pico_zense_undistorter.hpp"
using namespace cv;

// the unit of maximum_depth_for_undistortion is [mm]
PicoZenseUndistorter::PicoZenseUndistorter(
    ParameterManager cfgParam, std::string distortionKey,
    double _maximum_depth_for_undistortion /* = 1200*/) {
  maximum_depth_for_undistortion = _maximum_depth_for_undistortion;

  // Read undistortion paramters
  h5_param_path =
      cfgParam.readStringData(distortionKey, "undistortion_params_h5");
  h5io = cv::hdf::open(h5_param_path);

  load_undistortion_params();
  if (!undistortion_enabled) return;

  // set grid paramter
  yz_grid_scale = cfgParam.readDoubleData(distortionKey, "yz_grid_scale");
  x_grid_scale = cfgParam.readDoubleData(distortionKey, "x_grid_scale");

  x_min = cfgParam.readDoubleData(distortionKey, "x_min");
  x_max = cfgParam.readDoubleData(distortionKey, "x_max");
  y_min = cfgParam.readDoubleData(distortionKey, "y_min");
  y_max = cfgParam.readDoubleData(distortionKey, "y_max");
  z_min = cfgParam.readDoubleData(distortionKey, "z_min");
  z_max = cfgParam.readDoubleData(distortionKey, "z_max");

  x_nbin = int64_t((x_max - x_min) / x_grid_scale + 1e-3);
  y_nbin = int64_t((y_max - y_min) / yz_grid_scale + 1e-3);
  z_nbin = int64_t((z_max - z_min) / yz_grid_scale + 1e-3);
  x_grid_size = x_nbin + 1;
  yz_grid_size = y_nbin + 1;  // =z_nbin + 1
  yz_total_num_grid = yz_grid_size * yz_grid_size;

  std::cout << "Generating Hashmap ..." << std::endl;

  std::unordered_map<int64_t, double>
      _map_hash2ddcv_near;  // depth-distortion correction var
  std::unordered_map<int64_t, double>
      _map_hash2ddcv_mid;  // depth-distortion correction var
  std::unordered_map<int64_t, double>
      _map_hash2ddcv_far;  // depth-distortion correction var
  if (undistortion_availability_each_depth[0] == 1) {
    for (int i = 0; i < depth_distortion_coeffs[0].size(); i++) {
      _map_hash2ddcv_near.insert(std::pair<int64_t, double>(
          int64_t(hash_vector[0][i]), depth_distortion_coeffs[0][i]));
    }
  }
  if (undistortion_availability_each_depth[1] == 1) {
    for (int i = 0; i < depth_distortion_coeffs[1].size(); i++) {
      _map_hash2ddcv_mid.insert(std::pair<int64_t, double>(
          int64_t(hash_vector[1][i]), depth_distortion_coeffs[1][i]));
    }
  }
  if (undistortion_availability_each_depth[2] == 1) {
    for (int i = 0; i < depth_distortion_coeffs[2].size(); i++) {
      _map_hash2ddcv_far.insert(std::pair<int64_t, double>(
          int64_t(hash_vector[2][i]), depth_distortion_coeffs[2][i]));
    }
  }
  map_hash2ddcv.push_back(_map_hash2ddcv_near);
  map_hash2ddcv.push_back(_map_hash2ddcv_mid);
  map_hash2ddcv.push_back(_map_hash2ddcv_far);
  std::cout << "Finished" << std::endl;
}

void PicoZenseUndistorter::load_undistortion_params() {
  std::string depth_range_str = "near";
  _load_undistortion_params(depth_range_str);
  depth_range_str = "mid";
  _load_undistortion_params(depth_range_str);
  depth_range_str = "far";
  _load_undistortion_params(depth_range_str);

  // If no parameters are available on all distance range, undistortion process
  // will be skip
  if (undistortion_availability_each_depth.any()) {
    undistortion_enabled = true;
  } else {
    undistortion_enabled = false;
    std::cerr << 
          "Distortion correction data not found. Please check file path and "
          "contents of data."
        << std::endl;
  }
}

void PicoZenseUndistorter::_load_undistortion_params(
    std::string depth_range_str) {
  std::string _h5_depth_path = "/" + depth_range_str;

  std::vector<double> _depth_distortion_coeffs;
  std::vector<double> _hash_vector;
  cv::Mat _hashlist, _weight_buff;

  bool depth_range_existance = h5io->hlexists(_h5_depth_path);
  if (depth_range_existance) {
    if (depth_range_str == "near") {
      undistortion_availability_each_depth.set(0, 1);
      std::cout << "Near Range Undistortion enabled!" << std::endl;
    } else if (depth_range_str == "mid") {
      undistortion_availability_each_depth.set(1, 1);
      std::cout << "Mid Range Undistortion enabled!" << std::endl;
    } else if (depth_range_str == "far") {
      undistortion_availability_each_depth.set(2, 1);
      std::cout << "Far Range Undistortion enabled!" << std::endl;
    }
    h5io->dsread(_hashlist, _h5_depth_path + "/hashlist");
    h5io->dsread(_weight_buff, _h5_depth_path + "/weights");
    _depth_distortion_coeffs.assign((double *)_weight_buff.datastart,
                                    (double *)_weight_buff.dataend);
    _hash_vector.assign((double *)_hashlist.datastart,
                        (double *)_hashlist.dataend);
  }
  depth_distortion_coeffs.push_back(_depth_distortion_coeffs);
  hash_vector.push_back(_hash_vector);
}

void PicoZenseUndistorter::undistortion(cv::Mat &src, CameraParameter camera_parameter,
                                        int32_t depth_range) 
{
  // When if undistortion parameter is not registerered on input depth_range,
  // undistortion process will be skip
  if (!(undistortion_enabled) |
      (undistortion_availability_each_depth[depth_range] == 0))
    return;
  double fw = camera_parameter.fx;
  double cw = camera_parameter.cx;
  double fh = camera_parameter.fy;
  double ch = camera_parameter.cy;

  for (int h = 0; h < src.rows; h++) {
    for (int w = 0; w < src.cols; w++) {
      unsigned short depth_value_short = src.at<short>(h, w);

      if ((depth_value_short == 0) ||
          (depth_value_short >= maximum_depth_for_undistortion) ||
          !std::isfinite(depth_value_short))
        continue;
      double depth_value_double = (double)(depth_value_short) / 1000;

      double _x = depth_value_double;                   // x:depth-axis
      double _y = -depth_value_double * (h - ch) / fh;  // y:vertical-axis
      double _z = depth_value_double * (w - cw) / fw;   // z:horisontal-axis

      if (_x > x_max) {
        _x = x_max;
      }
      if (_y > y_max) {
        _y = y_max;
      }
      if (_z > y_max) {
        _z = y_max;
      }

      int64_t _x_hash = int64_t((_x - x_min) / x_grid_scale + 1e-3) *
                        yz_grid_size * yz_grid_size;
      int64_t _y_hash =
          int64_t((_y - y_min) / yz_grid_scale + 1e-3) * yz_grid_size;
      int64_t _z_hash = int64_t((_z - z_min) / yz_grid_scale + 1e-3);
      int64_t _hash = _z_hash + _y_hash + _x_hash;

      std::unordered_map<int64_t, double>::iterator it =
          map_hash2ddcv[depth_range].find(_hash);
      if (it != map_hash2ddcv[depth_range].end()) {  // component found
        double ddcv = it->second;
        src.at<short>(h, w) = src.at<short>(h, w) + (short)(ddcv * 1000);
      }
    }
  }
}
