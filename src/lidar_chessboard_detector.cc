#include "lidar_camera_calibration/lidar_chessboard_detector.h"

LidarChessboardDetector::LidarChessboardDetector(ros::NodeHandle nh,
                                                 ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh) {
  pnh_.param<std::string>("common/button_command_topic", button_command_topic_,
                          "");
  pnh_.param<std::string>("lidar_chessboard_detector/raw_cloud_topic",
                          raw_cloud_topic_, "");
  pnh_.param<std::string>("lidar_chessboard_detector/candidate_cloud_topic",
                          candidate_cloud_topic_, "");
  pnh_.param<std::string>("lidar_chessboard_detector/chessboard_cloud_topic",
                          chessboard_cloud_topic_, "");
  pnh_.param<std::string>("common/frame_id", frame_id_, "");
  pnh_.param<std::string>("common/data_path", data_path_, "");
  pnh_.param<double>("lidar_chessboard_detector/min_height", min_height_, 0.0);
  pnh_.param<double>("lidar_chessboard_detector/max_height", max_height_, 0.0);
  pnh_.param<double>("lidar_chessboard_detector/min_angle", min_angle_, 0.0);
  pnh_.param<double>("lidar_chessboard_detector/max_angle", max_angle_, 0.0);
  pnh_.param<double>("lidar_chessboard_detector/min_range", min_range_, 0.0);
  pnh_.param<double>("lidar_chessboard_detector/max_range", max_range_, 0.0);
  pnh_.param<int>("lidar_chessboard_detector/max_iterations", max_iterations_,
                  0);
  pnh_.param<double>("lidar_chessboard_detector/max_outlier_distance",
                     max_outlier_distance_, 0.0);
  pnh_.param<double>("lidar_chessboard_detector/cluster_tolerance",
                     cluster_tolerance_, 0.0);
  pnh_.param<int>("lidar_chessboard_detector/min_cluster_size",
                  min_cluster_size_, 0);
  pnh_.param<int>("lidar_chessboard_detector/max_cluster_size",
                  max_cluster_size_, 0);
  pnh_.param<int>("lidar_chessboard_detector/max_cluster_size",
                  max_cluster_size_, 0);
  pnh_.param<int>("common/lidar_chessboard_points_number",
                  chessboard_points_number_, 0);

  // Clear data
  button_command_.clear();
  is_detecting_ = false;

  // Initialize cloud ptr
  raw_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  chessboard_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // Initialize chessboard coefficients
  chessboard_coefficients_ =
      pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

  dr_cb_ = boost::bind(&LidarChessboardDetector::DynamicrReconfigureCallback,
                       this, _1, _2);
  dr_srv_.setCallback(dr_cb_);

  sub_button_command_ =
      nh_.subscribe(button_command_topic_, 2,
                    &LidarChessboardDetector::ButtonCommandCallback, this);

  pub_raw_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(raw_cloud_topic_, 2);
  pub_candidate_cloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>(candidate_cloud_topic_, 2);
  pub_chessboard_cloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>(chessboard_cloud_topic_, 2);
}

void LidarChessboardDetector::Run() {
  // Get all cloud files in given directory
  std::vector<std::string> raw_cloud_files =
      GetFilesInfolder(data_path_ + "cloud");

  // Chessborad plane model coefficients file
  std::ofstream chessboard_model_file(data_path_ +
                                      "result/lidar_chessboard_model.txt");
  chessboard_model_file << "index,alpha_x,alpha_y,alpha_z,theta" << std::endl;

  // Chessboard points file
  std::ofstream chessboard_points_file(data_path_ +
                                       "result/lidar_chessboard_points.txt");
  chessboard_points_file << "index,x,y,z" << std::endl;

  // Traverse all files
  for (int i = 0; i < raw_cloud_files.size(); ++i) {
    // Read raw cloud from file
    std::string raw_cloud_name = data_path_ + "cloud/" + raw_cloud_files[i];
    ROS_INFO("[Run] Processing %s", raw_cloud_name.c_str());
    raw_cloud_->clear();
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(raw_cloud_name, *raw_cloud_) ==
        -1) {
      ROS_ERROR("[Run] Cloud reading failed.");
      ros::shutdown();
    }

    // Detect chessboard
    if (!raw_cloud_->empty() && !is_detecting_) {
      is_detecting_ = true;
      chessboard_cloud_->clear();
      Detect(raw_cloud_, chessboard_cloud_);
      is_detecting_ = false;
    }

    // Wiating dynamic reconfigure
    while (button_command_ != "Next") {
      ros::spinOnce();
    }
    button_command_.clear();

    // Write results to file
    std::string index =
        raw_cloud_files[i].substr(0, raw_cloud_files[i].size() - 4);
    SaveChessboardmodel(index, chessboard_model_file);
    SaveChessboardPoints(index, chessboard_points_file);
    ROS_INFO("[Run] Save result to file");
  }
}

void LidarChessboardDetector::ButtonCommandCallback(
    const std_msgs::String::ConstPtr &in_command) {
  button_command_ = in_command->data;
}

void LidarChessboardDetector::DynamicrReconfigureCallback(
    lidar_camera_calibration::ChessboardDetectorConfig &config,
    uint32_t level) {
  // Get transform parameters
  min_height_ = config.min_height;
  max_height_ = config.max_height;
  min_angle_ = config.min_angle;
  max_angle_ = config.max_angle;
  min_range_ = config.min_range;
  max_range_ = config.max_range;
  max_iterations_ = config.max_iterations;
  max_outlier_distance_ = config.max_outlier_distance;
  cluster_tolerance_ = config.cluster_tolerance;
  min_cluster_size_ = config.min_cluster_size;
  max_cluster_size_ = config.max_cluster_size;

  // Detect chessboard
  if (!raw_cloud_->empty() && !is_detecting_) {
    is_detecting_ = true;
    chessboard_cloud_->clear();
    Detect(raw_cloud_, chessboard_cloud_);
    is_detecting_ = false;
  }
}

void LidarChessboardDetector::Detect(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr chessboard_cloud) {
  // Only remain front points
  pcl::PointCloud<pcl::PointXYZI>::Ptr candidate_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (!in_cloud->points.empty()) RemoveOutlier(in_cloud, candidate_cloud);

  // Find chessboard cloud using ransac
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (!candidate_cloud->points.empty())
    SegmentPlane(candidate_cloud, plane_cloud);

  // Find chessboard which has max points
  if (!plane_cloud->points.empty())
    FindMaxClsuter(plane_cloud, chessboard_cloud);

  // Publish raw cloud
  if (!in_cloud->points.empty()) {
    sensor_msgs::PointCloud2 raw_cloud_msg;
    pcl::toROSMsg(*in_cloud, raw_cloud_msg);
    raw_cloud_msg.header.frame_id = frame_id_;
    raw_cloud_msg.header.stamp = ros::Time::now();
    pub_raw_cloud_.publish(raw_cloud_msg);
  }

  // Publish candidate cloud
  if (!chessboard_cloud->points.empty()) {
    sensor_msgs::PointCloud2 candidate_cloud_msg;
    pcl::toROSMsg(*candidate_cloud, candidate_cloud_msg);
    candidate_cloud_msg.header.frame_id = frame_id_;
    candidate_cloud_msg.header.stamp = ros::Time::now();
    pub_candidate_cloud_.publish(candidate_cloud_msg);
  }

  // Publish chessboard cloud
  if (!chessboard_cloud->points.empty()) {
    sensor_msgs::PointCloud2 chessboard_cloud_msg;
    pcl::toROSMsg(*chessboard_cloud, chessboard_cloud_msg);
    chessboard_cloud_msg.header.frame_id = frame_id_;
    chessboard_cloud_msg.header.stamp = ros::Time::now();
    pub_chessboard_cloud_.publish(chessboard_cloud_msg);
  }
}

void LidarChessboardDetector::RemoveOutlier(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud) {
  for (auto pt : in_cloud->points) {
    // Remove points two low or two high
    if (pt.z < min_height_ || pt.z > max_height_) continue;
    // Remove points too far away from lidar origin
    float range = sqrt(pt.x * pt.x + pt.y * pt.y);
    if (range < min_range_ || range > max_range_) continue;
    // Remove points out of camera view
    float angle = atan2(pt.y, pt.x) * 180 / M_PI;
    if (angle < 0) angle += 360;
    if (angle < min_angle_ || angle > max_angle_) continue;
    out_cloud->points.push_back(pt);
  }
}

void LidarChessboardDetector::SegmentPlane(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud) {
  // Find chessboard cloud using ransac
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iterations_);
  seg.setDistanceThreshold(max_outlier_distance_);
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(in_cloud);
  seg.segment(*inliers, *chessboard_coefficients_);
  if (inliers->indices.size() == 0) {
    ROS_WARN("Could not estimate a planar model for the given dataset.");
  }

  // Extract chessboard cloud
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);  // true removes the indices
  extract.filter(*out_cloud);
}

void LidarChessboardDetector::FindMaxClsuter(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud) {
  // Build kd-tree for cloud
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(in_cloud);

  // Euclidean cluster
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(in_cloud);
  ec.extract(cluster_indices);

  // Find cluster with max points
  int max_num = 0;
  int max_index = 0;
  for (int i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.size() > max_num) {
      max_num = cluster_indices[i].indices.size();
      max_index = i;
    }
  }

  // Extract chessboard cloud
  pcl::PointIndices max_indices = cluster_indices[max_index];
  for (int i = 0; i < max_indices.indices.size(); ++i) {
    out_cloud->points.push_back(in_cloud->points[max_indices.indices[i]]);
  }
}

void LidarChessboardDetector::SaveChessboardmodel(const std::string &index,
                                                  std::ofstream &file) {
  file << index << "," << chessboard_coefficients_->values[0] << ","
       << chessboard_coefficients_->values[1] << ","
       << chessboard_coefficients_->values[2] << ","
       << chessboard_coefficients_->values[3] << std::endl;
}

void LidarChessboardDetector::SaveChessboardPoints(const std::string &index,
                                                   std::ofstream &file) {
  // Random sample for chessboard cloud
  pcl::RandomSample<pcl::PointXYZI> random_sample;
  pcl::PointCloud<pcl::PointXYZI>::Ptr sample_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  random_sample.setInputCloud(chessboard_cloud_);
  random_sample.setSample(chessboard_points_number_);
  random_sample.filter(*sample_cloud);

  if (sample_cloud->points.size() != chessboard_points_number_) {
    ROS_ERROR(
        "[SaveChessboardPoints] Chessboard points size is %ld not equal to "
        "requared size %d",
        sample_cloud->points.size(), chessboard_points_number_);
    ros::shutdown();
  }

  // Save sample points to file
  for (const auto pt : sample_cloud->points) {
    file << index << "," << pt.x << "," << pt.y << "," << pt.z << std::endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_chessboard_detector");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  LidarChessboardDetector lidar_chessboard_detector(nh, pnh);

  // Sleep some seconds waiting for data publish
  ros::Duration(5).sleep();
  lidar_chessboard_detector.Run();

  return 0;
}
