#include "lidar_camera_calibration/manual_calibration.h"

ManualCalibration::ManualCalibration(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh) {
  pnh_.param<std::string>("in_cloud_topic", in_cloud_topic_, "");
  pnh_.param<std::string>("in_image_topic", in_image_topic_, "");
  pnh_.param<std::string>("out_image_topic", out_image_topic_, "");
  pnh_.param<std::string>("out_cloud_topic", out_cloud_topic_, "");
  pnh_.param<std::string>("data_path", data_path_, "");
  pnh_.param<double>("min_angle", min_angle_, 240.0);
  pnh_.param<double>("max_angle", max_angle_, 300.0);
  pnh_.param<double>("min_range", min_range_, 2.0);
  pnh_.param<double>("max_range", max_range_, 100.0);

  // Read calibration parameters from file
  ReadCalibrationResults();

  // initialize cloud ptr
  raw_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // Clear project flag
  is_project_ = false;

  dr_cb_ =
      boost::bind(&ManualCalibration::DynamicReconfigureCallback, this, _1, _2);
  dr_srv_.setCallback(dr_cb_);

  sub_cloud_ = nh_.subscribe(in_cloud_topic_, 2,
                             &ManualCalibration::PointCloudCallback, this);
  sub_image_ = nh_.subscribe(in_image_topic_, 2,
                             &ManualCalibration::ImageCallback, this);

  pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(out_cloud_topic_, 2);
  image_transport::ImageTransport it(nh_);
  pub_image_ = it.advertise(out_image_topic_, 2);
}

ManualCalibration::~ManualCalibration() {
  // write calibration results to file
  WriteCalibrationResults();
}

void ManualCalibration::PointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_in) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_in, *cloud_ptr);

  // Remove point cloud out of camera view
  raw_cloud_->points.clear();
  ClipCameraCloud(cloud_ptr, raw_cloud_, min_range_, max_range_, min_angle_,
                  max_angle_);

  // Publish in camera cloud
  sensor_msgs::PointCloud2 out_cloud_msg;
  pcl::toROSMsg(*raw_cloud_, out_cloud_msg);
  out_cloud_msg.header = cloud_in->header;
  pub_cloud_.publish(out_cloud_msg);

  if (!is_project_ && !raw_cloud_->empty() && !raw_image_.empty()) {
    is_project_ = true;
    // Project cloud onto image
    double start_time = ros::Time::now().toSec();
    cv::Mat projected_image = ProjectCloudToImage(
        raw_cloud_, raw_image_, transform_matrix_, intrinsic_matrix_,
        distortion_coeffs_, min_range_, max_range_);
    // Publish projected image
    sensor_msgs::ImagePtr image_msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", projected_image)
            .toImageMsg();
    pub_image_.publish(image_msg);
    is_project_ = false;
  }
}

void ManualCalibration::ImageCallback(
    const sensor_msgs::ImageConstPtr& image_in) {
  raw_image_ = cv_bridge::toCvShare(image_in, "8UC3")->image;
}

void ManualCalibration::DynamicReconfigureCallback(
    lidar_camera_calibration::ManualCalibrationConfig& config, uint32_t level) {
  // Get transform parameters
  Eigen::Vector3d euler_angles(config.yaw, config.pitch, config.roll);
  Eigen::Vector3d translation_vector(config.tx, config.ty, config.tz);
  transform_matrix_ =
      EulerAnglesToTransformAffine(euler_angles, translation_vector).matrix();

  if (!is_project_ && !raw_cloud_->empty() && !raw_image_.empty()) {
    is_project_ = true;
    // Project cloud onto image
    cv::Mat projected_image = ProjectCloudToImage(
        raw_cloud_, raw_image_, transform_matrix_, intrinsic_matrix_,
        distortion_coeffs_, min_range_, max_range_);
    // Publish projected image
    sensor_msgs::ImagePtr image_msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", projected_image)
            .toImageMsg();
    pub_image_.publish(image_msg);
    is_project_ = false;
  }
}

void ManualCalibration::WriteCalibrationResults() {
  // Separate transform matrix to euler angles and translation vector
  Eigen::Vector3d euler_angles;
  Eigen::Vector3d translation_vector;
  TransformMatrixToEulerAngles(transform_matrix_, euler_angles,
                               translation_vector);

  // Write transform parmeters to file
  std::string result_file_path =
      data_path_ + "result/lidar_camera_parameters_manual.txt";
  std::ofstream result_file(result_file_path);
  // Write euler angles
  result_file << "euler_angles";
  for (int i = 0; i < euler_angles.size(); ++i)
    result_file << "," << euler_angles(i);
  result_file << std::endl;
  // Write translation vector
  result_file << "translation_vector";
  for (int i = 0; i < translation_vector.size(); ++i)
    result_file << "," << translation_vector(i);
  result_file << std::endl;

  // Write camera parameters file
  std::string camera_params_path = data_path_ + "result/camera_parameters.txt";
  std::ifstream camera_params_file;
  camera_params_file.open(camera_params_path.c_str(), std::ios::in);
  if (!camera_params_file) {
    std::cout << "[WriteCalibrationResults]: File " << camera_params_path
              << " does not exist" << std::endl;
    ros::shutdown();
  }
  result_file << camera_params_file.rdbuf();

  // Close all files
  camera_params_file.close();
  result_file.close();

  std::cout << "[WriteCalibrationResults] Save calibration result to "
            << result_file_path << std::endl;
}

void ManualCalibration::ReadCalibrationResults() {
  // Open calibration result file
  std::string calibration_result_path =
      data_path_ + "result/lidar_camera_parameters.txt";
  std::ifstream calibration_result_file;
  calibration_result_file.open(calibration_result_path.c_str(), std::ios::in);
  if (!calibration_result_file) {
    ROS_ERROR("[ReadCalibrationResults]: File %s does not exist",
              calibration_result_path.c_str());
    ros::shutdown();
  }

  // Read euler angles
  std::vector<double> euler_angles_array;
  ReadLine<double>(calibration_result_file, 3, NULL, &euler_angles_array);
  Eigen::Vector3d euler_angles = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      euler_angles_array.data(), 3);

  // Read translation vector
  std::vector<double> translation_vector_array;
  ReadLine<double>(calibration_result_file, 3, NULL, &translation_vector_array);
  Eigen::Vector3d translation_vector =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
          translation_vector_array.data(), 3);

  // Combine euler angles and translation vector
  transform_matrix_ =
      EulerAnglesToTransformAffine(euler_angles, translation_vector).matrix();

  // Read camera intrinsics matrix
  std::vector<double> intrinsic_matrix_array;
  ReadLine<double>(calibration_result_file, 9, NULL, &intrinsic_matrix_array);
  intrinsic_matrix_ =
      Eigen::Map<Eigen::Matrix<double, 3, 3> >(intrinsic_matrix_array.data())
          .transpose();

  // Read camera distortion coefficients
  std::vector<double> distortion_coefficients_array;
  ReadLine<double>(calibration_result_file, 5, NULL,
                   &distortion_coefficients_array);
  distortion_coeffs_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      distortion_coefficients_array.data(), 5);

  // Close camera parameters file
  calibration_result_file.close();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "manual_calibration");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ManualCalibration manual_calibration(nh, pnh);

  ros::spin();

  return 0;
}
