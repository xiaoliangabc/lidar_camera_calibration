#ifndef LIDAR_CAMERA_CALIBRATION_MANUAL_CALIBRATION_NODE_H_
#define LIDAR_CAMERA_CALIBRATION_MANUAL_CALIBRATION_NODE_H_

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include <lidar_camera_calibration/transformationConfig.h>

class ManualCalibrationNode {
 public:
  ManualCalibrationNode();

  // receive point cloud
  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in);

  // receive image
  void ImageCallback(const sensor_msgs::CompressedImageConstPtr& image_in);

  // get dynamic parameters
  void ConfigCallback(lidar_camera_calibration::transformationConfig& config,
                      uint32_t level);

 private:
  ros::NodeHandle nh_;

  // subscriber
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_image_;

  // publisher
  ros::Publisher pub_cloud_;
  image_transport::Publisher pub_image_;

  // topic name
  std::string in_cloud_topic_;
  std::string in_image_topic_;
  std::string out_image_topic_;
  std::string out_cloud_topic_;

  // server parameters
  dynamic_reconfigure::Server<lidar_camera_calibration::transformationConfig>
      dr_srv_;
  dynamic_reconfigure::Server<
      lidar_camera_calibration::transformationConfig>::CallbackType dr_cb_;

  // cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;

  // image
  cv::Mat image_;

  // tranform params from lidar to camera
  Eigen::Affine3f trans_lidar_to_camera_;
  double roll_;   // radian
  double pitch_;  // radian
  double yaw_;    // radian
  double tx_;
  double ty_;
  double tz_;

  // transform params from camera to image
  Eigen::MatrixXf trans_camera_to_image_;
  cv::Mat intrinsic_matrix_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;

  // camera distortion coefficients
  cv::Mat distortion_coeffs_;
  double k1_;
  double k2_;
  double k3_;
  double p1_;
  double p2_;

  // outlier parameters
  double min_theta_;
  double max_theta_;
  double min_range_;
  double max_range_;

  // project flag
  bool is_project_;

  // project cloud to image plane
  void Cloud2Image(pcl::PointCloud<pcl::PointXYZI> in_cloud, cv::Mat in_image,
                   Eigen::MatrixXf trans_lidar_to_image);

  // project cloud to image
  void Project(const pcl::PointCloud<pcl::PointXYZI>& in_cloud,
               const Eigen::MatrixXf& trans_lidar_to_image, cv::Mat& in_image);

  // remove outlier points
  void RemoveOutlier(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud);
};

#endif  // LIDAR_CAMERA_CALIBRATION_MANUAL_CALIBRATION_NODE_H_
