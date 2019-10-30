#ifndef LIDAR_CAMERA_CALIBRATION_CAMERA_CALIBRATION_NODE_H_
#define LIDAR_CAMERA_CALIBRATION_CAMERA_CALIBRATION_NODE_H_

#include <ros/ros.h>

#include <dirent.h>
#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/PointStamped.h>

class CameraCalibrationNode {
 public:
  CameraCalibrationNode();

  // receive point cloud
  void Calibration();

 private:
  ros::NodeHandle nh_;

  // file path
  std::string file_path_;

  // pattern size
  int pattern_width_;
  int pattern_height_;

  // square size
  int square_size_;

  // get all files in directory
  std::vector<std::string> get_files(std::string path);
};

#endif  // LIDAR_CAMERA_CALIBRATION_CAMERA_CALIBRATION_NODE_H_
