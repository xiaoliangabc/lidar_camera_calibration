#ifndef LIDAR_CAMERA_CALIBRATION_LIDAR_CAMERA_CALIBRATION_H_
#define LIDAR_CAMERA_CALIBRATION_LIDAR_CAMERA_CALIBRATION_H_

#include <ros/ros.h>

#include <dirent.h>
#include <fstream>
#include <iostream>

#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include "lidar_camera_optimization.h"
#include "transform_utils.h"
#include "utils.h"

class LidarCameraCalibration {
 public:
  LidarCameraCalibration(ros::NodeHandle nh, ros::NodeHandle pnh);

  // Calibrate lidar and camera
  void Calibration();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // File path
  std::string data_path_;

  // Lidar chessboard points number
  int lidar_chessboard_points_number_;

  // Clip cloud params
  double min_angle_;
  double max_angle_;
  double min_range_;
  double max_range_;

  // Parse camera chessboard plane model coefficients form file
  void CameraPlaneModel(Eigen::MatrixXd& camera_planes,
                        std::vector<int>& valid_indices);

  // Parse lidar chessboard plane model coefficients form file
  void LidarPlaneModel(Eigen::MatrixXd& lidar_planes,
                       const std::vector<int>& valid_indices);

  // Initial estimate of the transformation by estimating the translation part
  // and the rotation part independently
  Eigen::Matrix4d InitialEstimate(const Eigen::MatrixXd& camera_planes,
                                  const Eigen::MatrixXd& lidar_planes);

  // Iterative optimization procedure with initial estimates
  Eigen::Matrix4d IterativeOptimization(const Eigen::MatrixXd& camera_planes,
                                        const Eigen::Matrix4d& initial_trasform,
                                        const std::vector<int>& valid_indices);

  // Display calibration result by prject lidar points to image
  void DisplayCalibrationResult(const Eigen::Matrix4d& optimization_trasform);

  // Read camera parameters form file
  void ReadCameraparams(Eigen::MatrixXd& intrinsics_matrix,
                        Eigen::VectorXd& distortion_coefficients,
                        cv::Size* image_size);

  // Save calibration result
  void SaveCalibrationResult(const Eigen::Matrix4d& optimization_trasform);
};

#endif  // LIDAR_CAMERA_CALIBRATION_CAMERA_CALIBRATION_NODE_H_
