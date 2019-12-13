#ifndef LIDAR_CAMERA_CALIBRATION_DATA_GENERATE_H_
#define LIDAR_CAMERA_CALIBRATION_DATA_GENERATE_H_

#include <ros/ros.h>

#include <iomanip>
#include <iostream>
#include <sstream>

#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <std_msgs/String.h>

class DataGenerate {
 public:
  DataGenerate(ros::NodeHandle nh, ros::NodeHandle pnh);

  // Receive point cloud
  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in);

  // Receive image
  void ImageCallback(const sensor_msgs::ImageConstPtr& image_in);

  // Receive button command
  void ButtonCommandCallback(const std_msgs::String::ConstPtr& in_command);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscriber
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_image_;
  ros::Subscriber sub_button_command_;

  // Topic names
  std::string cloud_topic_;
  std::string image_topic_;
  std::string button_command_topic_;

  // Cloud
  pcl::PointCloud<pcl::PointXYZI> cloud_;

  // Image
  cv::Mat image_;

  // Starting file numbers
  int starting_file_num_;

  // Data path to save cloud and image
  std::string data_path_;
};

#endif  // LIDAR_CAMERA_CALIBRATION_DATA_GENERATE_NODE_H_
