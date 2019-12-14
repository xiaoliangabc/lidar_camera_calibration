#ifndef LIDAR_CAMERA_CALIBRATION_LIDAR_CAMERA_OPTIMIZATION_H_
#define LIDAR_CAMERA_CALIBRATION_LIDAR_CAMERA_OPTIMIZATION_H_

#include <ros/ros.h>

#include <fstream>

#include "cppoptlib/meta.h"
#include "cppoptlib/problem.h"
#include "cppoptlib/solver/bfgssolver.h"

#include "transform_utils.h"
#include "utils.h"

using namespace cppoptlib;

class LidarCameraOptimization : public Problem<double> {
 public:
  LidarCameraOptimization(std::string data_path,
                          int lidar_chessboard_points_number,
                          std::vector<int> valid_indices,
                          Eigen::MatrixXd camera_planes);

  double value(const Eigen::VectorXd &tranform);

 private:
  // Lidar chessboard points
  std::vector<Eigen::MatrixXd> lidar_chessboard_points_;

  // Camera chessboard plane model
  Eigen::MatrixXd camera_theta_;
  Eigen::MatrixXd camera_alpha_;
};

#endif  // LIDAR_CAMERA_CALIBRATION_LIDAR_CAMERA_OPTIMIZATION_H_
