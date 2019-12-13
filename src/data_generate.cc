#include "lidar_camera_calibration/data_generate.h"

DataGenerate::DataGenerate(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh) {
  pnh_.param<std::string>("cloud_topic", cloud_topic_, "");
  pnh_.param<std::string>("image_topic", image_topic_, "");
  pnh_.param<std::string>("button_command_topic", button_command_topic_, "");
  pnh_.param<std::string>("data_path", data_path_, "");
  pnh_.param<int>("starting_file_num", starting_file_num_, 0);

  sub_cloud_ =
      nh_.subscribe(cloud_topic_, 2, &DataGenerate::PointCloudCallback, this);
  sub_image_ =
      nh_.subscribe(image_topic_, 2, &DataGenerate::ImageCallback, this);
  sub_button_command_ = nh_.subscribe(
      button_command_topic_, 2, &DataGenerate::ButtonCommandCallback, this);
}

void DataGenerate::PointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_in) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_in, cloud_);
}

void DataGenerate::ImageCallback(const sensor_msgs::ImageConstPtr& image_in) {
  image_ = cv_bridge::toCvShare(image_in, "8UC3")->image;
}

void DataGenerate::ButtonCommandCallback(
    const std_msgs::String::ConstPtr& in_command) {
  if (in_command->data == "Save") {
    // Set file name
    std::ostringstream file_name;
    file_name << std::setfill('0') << std::setw(4) << starting_file_num_;
    starting_file_num_ += 1;

    // Save point cloud
    if (cloud_.empty()) {
      ROS_ERROR("[ButtonCommandCallback] Empty cloud");
      ros::shutdown();
    } else {
      std::string cloud_file = data_path_ + "cloud/" + file_name.str() + ".pcd";
      std::cout << pcl::io::savePCDFileASCII(cloud_file, cloud_);
      if (pcl::io::savePCDFileASCII(cloud_file, cloud_) >= 0) {
        ROS_INFO("[ButtonCommandCallback] Save cloud to %s",
                 cloud_file.c_str());
      } else {
        ROS_ERROR("[ButtonCommandCallback] Can not same cloud");
        ros::shutdown();
      }
    }

    // Save image
    if (image_.empty()) {
      ROS_ERROR("[ButtonCommandCallback] Empty image");
      ros::shutdown();
    } else {
      std::string image_file = data_path_ + "image/" + file_name.str() + ".jpg";
      // Without compression
      std::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
      compression_params.push_back(100);
      if (cv::imwrite(image_file, image_, compression_params)) {
        ROS_INFO("[ButtonCommandCallback] Save image to %s",
                 image_file.c_str());
      } else {
        ROS_ERROR("[ButtonCommandCallback] Can not same image");
        ros::shutdown();
      }
    }
  } else {
    ROS_WARN("[ButtonCommandCallback] invalid button command");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_generate");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  DataGenerate data_generate(nh, pnh);

  ros::spin();

  return 0;
}
