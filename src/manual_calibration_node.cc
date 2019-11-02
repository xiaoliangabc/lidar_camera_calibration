#include "lidar_camera_calibration/manual_calibration_node.h"

ManualCalibrationNode::ManualCalibrationNode() : nh_("~") {
  nh_.param<std::string>("in_cloud_topic", in_cloud_topic_, "");
  nh_.param<std::string>("in_image_topic", in_image_topic_, "");
  nh_.param<std::string>("out_image_topic", out_image_topic_, "");
  nh_.param<std::string>("out_cloud_topic", out_cloud_topic_, "");
  nh_.param<double>("min_theta", min_theta_, 240.0);
  nh_.param<double>("max_theta", max_theta_, 300.0);
  nh_.param<double>("min_range", min_range_, 2.0);
  nh_.param<double>("max_range", max_range_, 100.0);
  nh_.param<double>("yaw", yaw_, 0.0);
  nh_.param<double>("pitch", pitch_, 0.0);
  nh_.param<double>("roll", roll_, 0.0);
  nh_.param<double>("tx", tx_, 0.0);
  nh_.param<double>("ty", ty_, 0.0);
  nh_.param<double>("tz", tz_, 0.0);
  nh_.param<double>("fx", fx_, 0.0);
  nh_.param<double>("fy", fy_, 0.0);
  nh_.param<double>("cx", cx_, 0.0);
  nh_.param<double>("cy", cy_, 0.0);
  nh_.param<double>("k1", k1_, 0.0);
  nh_.param<double>("k2", k2_, 0.0);
  nh_.param<double>("k3", k3_, 0.0);
  nh_.param<double>("p1", p1_, 0.0);
  nh_.param<double>("p2", p2_, 0.0);

  trans_lidar_to_camera_ = Eigen::Affine3f::Identity();
  trans_lidar_to_camera_.translation() << tx_, ty_, tz_;
  trans_lidar_to_camera_.rotate(
      Eigen::AngleAxisf(yaw_, Eigen::Vector3f::UnitZ()));
  trans_lidar_to_camera_.rotate(
      Eigen::AngleAxisf(pitch_, Eigen::Vector3f::UnitY()));
  trans_lidar_to_camera_.rotate(
      Eigen::AngleAxisf(roll_, Eigen::Vector3f::UnitX()));

  // transform matrix from camera to image (eigen format)
  trans_camera_to_image_ = Eigen::MatrixXf::Zero(3, 4);
  trans_camera_to_image_ << fx_, 0, cx_, 0, 0, fy_, cy_, 0, 0, 0, 1, 0;

  // cameara intrinsic matrix (opencv format)
  intrinsic_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  intrinsic_matrix_.at<double>(0, 0) = fx_;
  intrinsic_matrix_.at<double>(0, 2) = cx_;
  intrinsic_matrix_.at<double>(1, 1) = fy_;
  intrinsic_matrix_.at<double>(1, 2) = cy_;

  // cameara distortion coefficients
  distortion_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
  distortion_coeffs_.at<double>(0, 0) = k1_;
  distortion_coeffs_.at<double>(1, 0) = k2_;
  distortion_coeffs_.at<double>(2, 0) = k3_;
  distortion_coeffs_.at<double>(3, 0) = p1_;
  distortion_coeffs_.at<double>(4, 0) = p2_;

  // initialize cloud ptr
  cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // project flag
  is_project_ = false;

  // config callback
  dr_cb_ = boost::bind(&ManualCalibrationNode::ConfigCallback, this, _1, _2);
  dr_srv_.setCallback(dr_cb_);

  // subscriber callback
  sub_cloud_ = nh_.subscribe(in_cloud_topic_, 2,
                             &ManualCalibrationNode::PointCloudCallback, this);
  sub_image_ = nh_.subscribe(in_image_topic_, 2,
                             &ManualCalibrationNode::ImageCallback, this);

  // publish image
  image_transport::ImageTransport it(nh_);
  pub_image_ = it.advertise(out_image_topic_, 2);

  // publish cloud
  pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(out_cloud_topic_, 2);
}

void ManualCalibrationNode::PointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_in) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_in, *raw_cloud_ptr);

  // remove outlier points
  cloud_->clear();
  RemoveOutlier(raw_cloud_ptr, cloud_);

  // publish valid cloud
  sensor_msgs::PointCloud2 out_cloud_msg;
  pcl::toROSMsg(*cloud_, out_cloud_msg);
  out_cloud_msg.header = cloud_in->header;
  pub_cloud_.publish(out_cloud_msg);

  // project cloud onto image
  if (!is_project_) {
    is_project_ = true;
    Eigen::MatrixXf trans_lidar_to_image =
        trans_camera_to_image_ * trans_lidar_to_camera_.matrix();
    Cloud2Image(*cloud_, image_, trans_lidar_to_image);
    is_project_ = false;
  }
}

// receive image
void ManualCalibrationNode::ImageCallback(
    const sensor_msgs::CompressedImageConstPtr& image_in) {
  image_ = cv::imdecode(cv::Mat(image_in->data), CV_LOAD_IMAGE_UNCHANGED);
  // image_= cv_bridge::toCvShare(image_in, "8UC3")->image;
}

void ManualCalibrationNode::ConfigCallback(
    lidar_camera_calibration::transformationConfig& config, uint32_t level) {
  // get transform parameters
  roll_ = config.roll;
  pitch_ = config.pitch;
  yaw_ = config.yaw;
  tx_ = config.tx;
  ty_ = config.ty;
  tz_ = config.tz;
  fx_ = config.fx;
  fy_ = config.fy;
  cx_ = config.cx;
  cy_ = config.cy;

  // transform matrix from lidar to camera
  trans_lidar_to_camera_ = Eigen::Affine3f::Identity();
  trans_lidar_to_camera_.translation() << tx_, ty_, tz_;
  trans_lidar_to_camera_.rotate(
      Eigen::AngleAxisf(yaw_, Eigen::Vector3f::UnitZ()));
  trans_lidar_to_camera_.rotate(
      Eigen::AngleAxisf(pitch_, Eigen::Vector3f::UnitY()));
  trans_lidar_to_camera_.rotate(
      Eigen::AngleAxisf(roll_, Eigen::Vector3f::UnitX()));

  // transform matrix from camera to image (eigen format)
  trans_camera_to_image_ = Eigen::MatrixXf::Zero(3, 4);
  trans_camera_to_image_ << fx_, 0, cx_, 0, 0, fy_, cy_, 0, 0, 0, 1, 0;

  // cameara intrinsic matrix (opencv format)
  intrinsic_matrix_.at<double>(0, 0) = fx_;
  intrinsic_matrix_.at<double>(0, 2) = cx_;
  intrinsic_matrix_.at<double>(1, 1) = fy_;
  intrinsic_matrix_.at<double>(1, 2) = cy_;

  // project cloud onto image
  if (!is_project_) {
    is_project_ = true;
    Eigen::MatrixXf trans_lidar_to_image =
        trans_camera_to_image_ * trans_lidar_to_camera_.matrix();
    Cloud2Image(*cloud_, image_, trans_lidar_to_image);
    is_project_ = false;
  }
}

void ManualCalibrationNode::Cloud2Image(
    pcl::PointCloud<pcl::PointXYZI> in_cloud, cv::Mat in_image,
    Eigen::MatrixXf trans_lidar_to_image) {
  // assert data
  if (in_cloud.empty() || in_image.empty()) return;

  // undistort image
  cv::Mat undistort_image;
  undistort(in_image, undistort_image, intrinsic_matrix_, distortion_coeffs_);

  // project cloud onto image
  Project(in_cloud, trans_lidar_to_image, undistort_image);

  // publish projected image
  sensor_msgs::ImagePtr image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistort_image)
          .toImageMsg();
  pub_image_.publish(image_msg);
}

void ManualCalibrationNode::Project(
    const pcl::PointCloud<pcl::PointXYZI>& in_cloud,
    const Eigen::MatrixXf& trans_lidar_to_image, cv::Mat& in_image) {
  // Get image format
  int image_width = in_image.cols;
  int image_height = in_image.rows;

  // transform image form RGB space to HSV space
  cv::Mat hsv_image;
  cv::cvtColor(in_image, hsv_image, cv::COLOR_BGR2HSV);

  for (auto pt : in_cloud.points) {
    // transform cloud from velodyne coordinates to the image plane
    Eigen::Vector4f cloud_point(pt.x, pt.y, pt.z, 1);
    Eigen::Vector3f image_point = trans_lidar_to_image * cloud_point;
    image_point(0) = image_point(0) / image_point(2);
    image_point(1) = image_point(1) / image_point(2);

    // Check if image point is valid
    if ((image_point(0) >= 0 && image_point(0) < image_width) &&
        (image_point(1) >= 0 && image_point(1) < image_height)) {
      const int col = static_cast<int>(image_point(1));
      const int row = static_cast<int>(image_point(0));
      // draw a circle in image
      float range = sqrt(pt.x * pt.x + pt.y * pt.y) - min_range_;
      cv::circle(
          hsv_image, cv::Point(row, col), 2,
          cv::Scalar(static_cast<int>(range / max_range_ * 255), 255, 255));
    }
  }
  // transform back image form HSV space to RGB space
  cv::cvtColor(hsv_image, in_image, cv::COLOR_HSV2BGR);
}

void ManualCalibrationNode::RemoveOutlier(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud) {
  for (auto pt : in_cloud->points) {
    // // remove points too far away from lidar origin
    float range = sqrt(pt.x * pt.x + pt.y * pt.y);
    if (range < min_range_ || range > max_range_) continue;
    // remove points out of camera view
    float theta = atan2(pt.y, pt.x) * 180 / M_PI;
    if (theta < 0) theta += 360;
    if (theta < min_theta_ || theta > max_theta_) continue;
    out_cloud->points.push_back(pt);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "manual_calibration_node");
  ManualCalibrationNode manual_calibration_node;
  ros::spin();

  return 0;
}
