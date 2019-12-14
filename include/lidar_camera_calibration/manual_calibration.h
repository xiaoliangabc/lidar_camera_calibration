#ifndef LIDAR_CAMERA_CALIBRATION_MANUAL_CALIBRATION_H_
#define LIDAR_CAMERA_CALIBRATION_MANUAL_CALIBRATION_H_

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include <dynamic_reconfigure/server.h>
#include <lidar_camera_calibration/ManualCalibrationConfig.h>

#include "transform_utils.h"
#include "utils.h"

class ManualCalibration {
 public:
  ManualCalibration(ros::NodeHandle nh, ros::NodeHandle pnh);

  ~ManualCalibration();

  // Receive point cloud
  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_in);

  // Receive image
  void ImageCallback(const sensor_msgs::ImageConstPtr& image_in);

  // Get dynamic reconfigure parameters
  void DynamicReconfigureCallback(
      lidar_camera_calibration::ManualCalibrationConfig& config,
      uint32_t level);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscriber
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_image_;

  // Publisher
  ros::Publisher pub_cloud_;
  image_transport::Publisher pub_image_;

  // Topic name
  std::string in_cloud_topic_;
  std::string in_image_topic_;
  std::string out_image_topic_;
  std::string out_cloud_topic_;

  // Dynamic reconfigure server parameters
  dynamic_reconfigure::Server<lidar_camera_calibration::ManualCalibrationConfig>
      dr_srv_;
  dynamic_reconfigure::Server<
      lidar_camera_calibration::ManualCalibrationConfig>::CallbackType dr_cb_;

  // Raw cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_;

  // Raw image
  cv::Mat raw_image_;

  // Data path
  std::string data_path_;

  // Transform matrix from lidar to camera
  Eigen::Matrix4d transform_matrix_;

  // Transform matrix from camera to image
  Eigen::Matrix3d intrinsic_matrix_;

  // camera distortion coefficients
  Eigen::VectorXd distortion_coeffs_;

  // Clip cloud params
  double min_angle_;
  double max_angle_;
  double min_range_;
  double max_range_;

  // Project flag(mutex)
  bool is_project_;

  // project cloud to image plane
  void Cloud2Image(pcl::PointCloud<pcl::PointXYZI> in_cloud, cv::Mat in_image,
                   cv::Mat euler_angles, cv::Mat translation_vector,
                   cv::Mat intrinsic_matrix, cv::Mat distortion_coeffs);

  // project cloud to image
  void Project(const pcl::PointCloud<pcl::PointXYZI>& in_cloud,
               const cv::Mat extrinsic_matrix, const cv::Mat& intrinsic_matrix,
               cv::Mat& in_image);

  // remove outlier points
  void RemoveOutlier(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud);

  // Write calibration results to file
  void WriteCalibrationResults();

  // Read calibration results from file
  void ReadCalibrationResults();
};

#endif  // LIDAR_CAMERA_CALIBRATION_MANUAL_CALIBRATION_NODE_H_
