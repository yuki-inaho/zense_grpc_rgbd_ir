
#include "common.hpp"
#include "parameter_manager.hpp"

#include <bitset>
#include <opencv2/hdf/hdf5.hpp>
#include <opencv2/opencv.hpp>

class PicoZenseUndistorter {
 public:
  PicoZenseUndistorter(){};
  PicoZenseUndistorter(ParameterManager cfgParam, std::string distortionKey,
                       double _maximum_depth_for_undistortion = 1200);
  void undistortion(cv::Mat &src, CameraParameter camera_parameter,
                    int32_t depth_range);

 private:
  void load_undistortion_params();
  void _load_undistortion_params(std::string depth_range_str);

  std::string h5_param_path;
  cv::Ptr<cv::hdf::HDF5> h5io;
  double x_max, x_min, y_max, y_min, z_max, z_min, yz_grid_scale, x_grid_scale;
  int64_t x_nbin, y_nbin, z_nbin;
  int64_t x_grid_size, yz_grid_size, yz_total_num_grid;

  std::vector<std::unordered_map<int64_t, double>>
      map_hash2ddcv;  // depth-distortion correction var

  std::vector<std::vector<double>> depth_distortion_coeffs;
  std::vector<std::vector<double>> hash_vector;

  std::bitset<3> undistortion_availability_each_depth;

  double maximum_depth_for_undistortion;
  bool undistortion_enabled;
};
