#include "lidar_camera_calibration/data_generate_node.h"

DataGenerateNode::DataGenerateNode() : nh_("~") {
  nh_.param<std::string>("cloud_topic", cloud_topic_, "");
  nh_.param<std::string>("image_topic", image_topic_, "");
  nh_.param<std::string>("click_topic", click_topic_, "");
  nh_.param<std::string>("file_path", file_path_, "");
  nh_.param<int>("file_num", file_num_, 0);

  sub_cloud_ = nh_.subscribe(cloud_topic_, 2,
                             &DataGenerateNode::PointCloudCallback, this);
  sub_image_ =
      nh_.subscribe(image_topic_, 2, &DataGenerateNode::ImageCallback, this);
  sub_click_ =
      nh_.subscribe(click_topic_, 2, &DataGenerateNode::ClickCallback, this);
}

void DataGenerateNode::PointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_in) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_in, cloud_);
}

// receive image
void DataGenerateNode::ImageCallback(
    const sensor_msgs::ImageConstPtr& image_in) {
  // image_ = cv::imdecode(cv::Mat(image_in->data), CV_LOAD_IMAGE_UNCHANGED);
  image_= cv_bridge::toCvShare(image_in, "8UC3")->image;
}

void DataGenerateNode::ClickCallback(
    const geometry_msgs::PointStampedConstPtr& click_in) {
  std::ostringstream file_name;
  file_name << std::setfill('0') << std::setw(4) << file_num_;
  file_num_ += 1;

  // save point cloud
  if (cloud_.empty()) {
    ROS_ERROR("[DataGenerateNode::ClickCallback] empty cloud");
  } else {
    std::string cloud_file = file_path_ + "cloud/" + file_name.str() + ".pcd";
    pcl::io::savePCDFileASCII(cloud_file, cloud_);
    ROS_INFO("[DataGenerateNode::ClickCallback] save cloud to %s",
             cloud_file.c_str());
  }

  // save image
  if (image_.empty()) {
    ROS_ERROR("[DataGenerateNode::ClickCallback] empty image");
  } else {
    std::string image_file = file_path_ + "image/" + file_name.str() + ".png";
    cv::imwrite(image_file, image_);
    ROS_INFO("[DataGenerateNode::ClickCallback] save image to %s",
             image_file.c_str());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_generate_node");
  DataGenerateNode data_generate_node;
  ros::spin();

  return 0;
}
