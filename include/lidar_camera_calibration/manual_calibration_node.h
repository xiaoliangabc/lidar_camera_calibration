#ifndef LIDAR_CAMERA_CALIBRATION_MANUAL_CALIBRATION_NODE_H_
#define LIDAR_CAMERA_CALIBRATION_MANUAL_CALIBRATION_NODE_H_

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include <dynamic_reconfigure/server.h>
#include <lidar_camera_calibration/transformationConfig.h>

class ManualCalibrationNode {
 public:
  ManualCalibrationNode();

  ~ManualCalibrationNode();

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

  // calibration file path
  std::string calibration_file_;

  // rotation matrix from lidar to camera
  cv::Mat rotation_vector_;

  /// translation matrix from lidar to camera
  cv::Mat translation_vector_;

  // transform params from camera to image
  cv::Mat intrinsic_matrix_;

  // camera distortion coefficients
  cv::Mat distortion_coeffs_;

  // outlier parameters
  double min_theta_;
  double max_theta_;
  double min_range_;
  double max_range_;

  // project flag
  bool is_project_;

  // write calibration results flag
  bool is_write_results_;

  // project cloud to image plane
  void Cloud2Image(pcl::PointCloud<pcl::PointXYZI> in_cloud, cv::Mat in_image,
                   cv::Mat rotation_vector, cv::Mat translation_vector,
                   cv::Mat intrinsic_matrix, cv::Mat distortion_coeffs);

  // project cloud to image
  void Project(const pcl::PointCloud<pcl::PointXYZI>& in_cloud,
               const cv::Mat extrinsic_matrix, const cv::Mat& intrinsic_matrix,
               cv::Mat& in_image);

  // remove outlier points
  void RemoveOutlier(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud);

  // calculates tranform matrix given rotation vector and translation vector.
  cv::Mat TransformVectorToTransformMatrix(const cv::Mat& rotation_vector,
                                           cv::Mat& translation_vector);

  // calculates rotation matrix given rotation vector.
  cv::Mat RotationVectorToRotationMatrix(const cv::Mat& rotation_vector);

  // calculates rotation vector given rotation matrix.
  cv::Mat RotationMatrixToRotationVector(const cv::Mat& rotation_matrix);

  // write calibration results to yaml file
  void WriteCalibrationResults(const std::string& file_path,
                               const cv::Mat& rotation_vector,
                               const cv::Mat& translation_vector,
                               const cv::Mat& intrinsic_matrix,
                               const cv::Mat& distortion_coeffs);

  // read calibration results from yaml file
  void ReadCalibrationResults(const std::string& file_path,
                              cv::Mat* rotation_vector,
                              cv::Mat* translation_vector,
                              cv::Mat* intrinsic_matrix,
                              cv::Mat* distortion_coeffs);
};

#endif  // LIDAR_CAMERA_CALIBRATION_MANUAL_CALIBRATION_NODE_H_
