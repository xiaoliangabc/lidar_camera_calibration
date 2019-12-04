#include "lidar_camera_calibration/manual_calibration_node.h"

ManualCalibrationNode::ManualCalibrationNode() : nh_("~") {
  nh_.param<std::string>("in_cloud_topic", in_cloud_topic_, "");
  nh_.param<std::string>("in_image_topic", in_image_topic_, "");
  nh_.param<std::string>("out_image_topic", out_image_topic_, "");
  nh_.param<std::string>("out_cloud_topic", out_cloud_topic_, "");
  nh_.param<std::string>("calibration_file", calibration_file_, "");
  nh_.param<double>("min_theta", min_theta_, 240.0);
  nh_.param<double>("max_theta", max_theta_, 300.0);
  nh_.param<double>("min_range", min_range_, 2.0);
  nh_.param<double>("max_range", max_range_, 100.0);
  nh_.param<bool>("is_write_results", is_write_results_, false);

  // read calibration parameters from yaml file
  ReadCalibrationResults(calibration_file_, &rotation_vector_,
                         &translation_vector_, &intrinsic_matrix_,
                         &distortion_coeffs_);

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

ManualCalibrationNode::~ManualCalibrationNode() {
  // write calibration results to file
  if (is_write_results_) {
    WriteCalibrationResults(calibration_file_, rotation_vector_,
                            translation_vector_, intrinsic_matrix_,
                            distortion_coeffs_);
  }
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
    Cloud2Image(*cloud_, image_, rotation_vector_, translation_vector_,
                intrinsic_matrix_, distortion_coeffs_);
    is_project_ = false;
  }
}

void ManualCalibrationNode::ImageCallback(
    const sensor_msgs::CompressedImageConstPtr& image_in) {
  image_ = cv::imdecode(cv::Mat(image_in->data), CV_LOAD_IMAGE_UNCHANGED);
  // image_= cv_bridge::toCvShare(image_in, "8UC3")->image;
}

void ManualCalibrationNode::ConfigCallback(
    lidar_camera_calibration::ManualCalibrationConfig& config, uint32_t level) {
  // get transform parameters
  rotation_vector_ =
      (cv::Mat_<double>(3, 1) << config.roll, config.pitch, config.yaw);
  translation_vector_ =
      (cv::Mat_<double>(3, 1) << config.tx, config.ty, config.tz);
  intrinsic_matrix_ = (cv::Mat_<double>(3, 3) << config.fx, 0.0, config.cx, 0.0,
                       config.fy, config.cy, 0.0, 0.0, 1.0);

  // project cloud onto image
  if (!is_project_) {
    is_project_ = true;
    Cloud2Image(*cloud_, image_, rotation_vector_, translation_vector_,
                intrinsic_matrix_, distortion_coeffs_);
    is_project_ = false;
  }
}

void ManualCalibrationNode::Cloud2Image(
    pcl::PointCloud<pcl::PointXYZI> in_cloud, cv::Mat in_image,
    cv::Mat rotation_vector, cv::Mat translation_vector,
    cv::Mat intrinsic_matrix, cv::Mat distortion_coeffs) {
  // assert data
  if (in_cloud.empty() || in_image.empty()) return;

  // undistort image
  cv::Mat undistort_image;
  undistort(in_image, undistort_image, intrinsic_matrix, distortion_coeffs);

  // project cloud onto image
  cv::Mat extrinsic_matrix =
      TransformVectorToTransformMatrix(rotation_vector, translation_vector);
  Project(in_cloud, extrinsic_matrix, intrinsic_matrix, undistort_image);

  // publish projected image
  sensor_msgs::ImagePtr image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistort_image)
          .toImageMsg();
  pub_image_.publish(image_msg);
}

void ManualCalibrationNode::Project(
    const pcl::PointCloud<pcl::PointXYZI>& in_cloud,
    const cv::Mat extrinsic_matrix, const cv::Mat& intrinsic_matrix,
    cv::Mat& in_image) {
  // Get image format
  int image_width = in_image.cols;
  int image_height = in_image.rows;

  // transform image form RGB space to HSV space
  cv::Mat hsv_image;
  cv::cvtColor(in_image, hsv_image, cv::COLOR_BGR2HSV);

  for (auto pt : in_cloud.points) {
    // transform cloud from velodyne coordinates to the image plane
    cv::Mat cloud_point = (cv::Mat_<double>(4, 1) << pt.x, pt.y, pt.z, 1);
    // Eigen::Vector4f cloud_point(pt.x, pt.y, pt.z, 1);
    cv::Mat image_point = intrinsic_matrix * extrinsic_matrix * cloud_point;
    image_point.at<double>(0) =
        image_point.at<double>(0) / image_point.at<double>(2);
    image_point.at<double>(1) =
        image_point.at<double>(1) / image_point.at<double>(2);

    // Check if image point is valid
    if ((image_point.at<double>(0) >= 0 &&
         image_point.at<double>(0) < image_width) &&
        (image_point.at<double>(1) >= 0 &&
         image_point.at<double>(1) < image_height)) {
      const int col = static_cast<int>(image_point.at<double>(1));
      const int row = static_cast<int>(image_point.at<double>(0));
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

cv::Mat ManualCalibrationNode::TransformVectorToTransformMatrix(
    const cv::Mat& rotation_vector, cv::Mat& translation_vector) {
  cv::Mat transform_matrix = cv::Mat::eye(3, 4, CV_64F);

  // assign rotation matrix to transform matrix
  cv::Mat rotation_matrix = RotationVectorToRotationMatrix(rotation_vector);
  cv::Mat rotation_tmp = transform_matrix(cv::Rect(0, 0, 3, 3));
  rotation_matrix.copyTo(rotation_tmp);

  // assign translation matrix to transform matrix
  cv::Mat translation_tmp = transform_matrix.col(3);
  translation_vector.copyTo(translation_tmp);

  return transform_matrix;
}

cv::Mat ManualCalibrationNode::RotationVectorToRotationMatrix(
    const cv::Mat& rotation_vector) {
  // Calculate rotation about x axis
  cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0,
                 cos(rotation_vector.at<double>(0, 0)),
                 -sin(rotation_vector.at<double>(0, 0)), 0,
                 sin(rotation_vector.at<double>(0, 0)),
                 cos(rotation_vector.at<double>(0, 0)));

  // Calculate rotation about y axis
  cv::Mat R_y =
      (cv::Mat_<double>(3, 3) << cos(rotation_vector.at<double>(0, 1)), 0,
       sin(rotation_vector.at<double>(0, 1)), 0, 1, 0,
       -sin(rotation_vector.at<double>(0, 1)), 0,
       cos(rotation_vector.at<double>(0, 1)));

  // Calculate rotation about z axis
  cv::Mat R_z =
      (cv::Mat_<double>(3, 3) << cos(rotation_vector.at<double>(0, 2)),
       -sin(rotation_vector.at<double>(0, 2)), 0,
       sin(rotation_vector.at<double>(0, 2)),
       cos(rotation_vector.at<double>(0, 2)), 0, 0, 0, 1);

  // Combined rotation matrix
  cv::Mat R = R_z * R_y * R_x;

  return R;
}

cv::Mat ManualCalibrationNode::RotationMatrixToRotationVector(
    const cv::Mat& rotation_matrix) {
  float sy =
      sqrt(rotation_matrix.at<double>(0, 0) * rotation_matrix.at<double>(0, 0) +
           rotation_matrix.at<double>(1, 0) * rotation_matrix.at<double>(1, 0));

  bool singular = sy < 1e-6;

  float x, y, z;
  if (!singular) {
    x = atan2(rotation_matrix.at<double>(2, 1),
              rotation_matrix.at<double>(2, 2));
    y = atan2(-rotation_matrix.at<double>(2, 0), sy);
    z = atan2(rotation_matrix.at<double>(1, 0),
              rotation_matrix.at<double>(0, 0));
  } else {
    x = atan2(-rotation_matrix.at<double>(1, 2),
              rotation_matrix.at<double>(1, 1));
    y = atan2(-rotation_matrix.at<double>(2, 0), sy);
    z = 0;
  }
  return (cv::Mat_<double>(1, 3) << x, y, z);
}

void ManualCalibrationNode::WriteCalibrationResults(
    const std::string& file_path, const cv::Mat& rotation_vector,
    const cv::Mat& translation_vector, const cv::Mat& intrinsic_matrix,
    const cv::Mat& distortion_coeffs) {
  cv::FileStorage fs(file_path, cv::FileStorage::WRITE);
  fs << "rotation_vector" << rotation_vector;
  fs << "translation_vector" << translation_vector;
  fs << "intrinsic_matrix" << intrinsic_matrix;
  fs << "distortion_coeffs" << distortion_coeffs;
  fs.release();
}

void ManualCalibrationNode::ReadCalibrationResults(const std::string& file_path,
                                                   cv::Mat* rotation_vector,
                                                   cv::Mat* translation_vector,
                                                   cv::Mat* intrinsic_matrix,
                                                   cv::Mat* distortion_coeffs) {
  cv::FileStorage fs(file_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    ROS_ERROR("Cannot open file calibration file '%s'", file_path.c_str());
    ros::shutdown();
  }
  fs["rotation_vector"] >> *rotation_vector;
  fs["translation_vector"] >> *translation_vector;
  fs["intrinsic_matrix"] >> *intrinsic_matrix;
  fs["distortion_coeffs"] >> *distortion_coeffs;
  fs.release();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "manual_calibration_node");
  ManualCalibrationNode manual_calibration_node;
  ros::spin();

  return 0;
}
