#ifndef LIDAR_CAMERA_CALIBRATION_DATA_GENERATE_NODE_H_
#define LIDAR_CAMERA_CALIBRATION_DATA_GENERATE_NODE_H_

#include <ros/ros.h>

#include <iomanip>
#include <iostream>
#include <sstream>

#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/PointStamped.h>

class DataGenerateNode {
 public:
  DataGenerateNode();

  // receive point cloud
  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in);

  // receive image
  void ImageCallback(const sensor_msgs::ImageConstPtr& image_in);

  // receive click signal
  void ClickCallback(const geometry_msgs::PointStampedConstPtr& click_in);

 private:
  ros::NodeHandle nh_;

  // subscriber
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_image_;
  ros::Subscriber sub_click_;

  // topic name
  std::string cloud_topic_;
  std::string image_topic_;
  std::string click_topic_;

  // cloud
  pcl::PointCloud<pcl::PointXYZI> cloud_;

  // image
  cv::Mat image_;

  // file numbers
  int file_num_;

  // file path
  std::string file_path_;
};

#endif  // LIDAR_CAMERA_CALIBRATION_DATA_GENERATE_NODE_H_
