#ifndef LIDAR_CAMERA_CALIBRATION_LIDAR_CHESSBOARD_DETECTOR_H_
#define LIDAR_CAMERA_CALIBRATION_LIDAR_CHESSBOARD_DETECTOR_H_

#include <ros/ros.h>

#include <fstream>
#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>

#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <lidar_camera_calibration/ChessboardDetectorConfig.h>

#include "utils.h"

class LidarChessboardDetector {
 public:
  LidarChessboardDetector(ros::NodeHandle nh, ros::NodeHandle pnh);

  // Run
  void Run();

  // Receive button command
  void ButtonCommandCallback(const std_msgs::String::ConstPtr& in_command);

  // Get dynamic econfigure parameters
  void DynamicrReconfigureCallback(
      lidar_camera_calibration::ChessboardDetectorConfig& config,
      uint32_t level);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscriber
  ros::Subscriber sub_button_command_;

  // Publisher
  ros::Publisher pub_raw_cloud_;
  ros::Publisher pub_candidate_cloud_;
  ros::Publisher pub_chessboard_cloud_;

  // Topic name
  std::string button_command_topic_;
  std::string raw_cloud_topic_;
  std::string candidate_cloud_topic_;
  std::string chessboard_cloud_topic_;

  // Frame id
  std::string frame_id_;

  // Dynamic reconfigure server parameters
  dynamic_reconfigure::Server<
      lidar_camera_calibration::ChessboardDetectorConfig>
      dr_srv_;
  dynamic_reconfigure::Server<
      lidar_camera_calibration::ChessboardDetectorConfig>::CallbackType dr_cb_;

  // Raw cloud and chessboard_cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr candidate_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr chessboard_cloud_;

  // Button command
  std::string button_command_;

  // Clip cloud params
  double min_height_;
  double max_height_;
  double min_theta_;
  double max_theta_;
  double min_range_;
  double max_range_;

  // Ransac params
  int max_iterations_;
  double max_outlier_distance_;

  // Cluster params
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;

  // Data path
  std::string data_path_;

  // Chessboard plane model coefficients
  pcl::ModelCoefficients::Ptr chessboard_coefficients_;

  // Chessboard points number
  int chessboard_points_number_;

  // detecting flag(mutex)
  bool is_detecting_;

  // Detect chessboard
  void Detect(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr chessboard_cloud);

  // Remove points out of view
  void RemoveOutlier(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud);

  // Segment plane using ransac
  void SegmentPlane(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud);

  // Find max points number cluster
  void FindMaxClsuter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud);

  // Save chessboard model coefficients to file
  void SaveChessboardmodel(const std::string& index, std::ofstream& file);

  // Save chessboard points to file
  void SaveChessboardPoints(const std::string& index, std::ofstream& file);
};

#endif  // LIDAR_CAMERA_CALIBRATION_LIDAR_CHESSBOARD_DETECTOR_NODE_H_
