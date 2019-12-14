#ifndef LIDAR_CAMERA_CALIBRATION_UTILS_H_
#define LIDAR_CAMERA_CALIBRATION_UTILS_H_

#include <ros/ros.h>

#include <dirent.h>
#include <fstream>
#include <iostream>

#include <boost/lexical_cast.hpp>

#include <pcl_ros/point_cloud.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Get all files name given a folder
inline std::vector<std::string> GetFilesInfolder(std::string path) {
  DIR *dir;
  struct dirent *ent;
  std::vector<std::string> files;
  if ((dir = opendir(path.c_str())) != NULL) {
    // Print all the files and directories within directory
    while ((ent = readdir(dir)) != NULL) {
      if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0)
        continue;
      files.push_back(ent->d_name);
    }
    closedir(dir);
  } else {
    // Could not open directory
    ROS_ERROR(" Could not open directory %s", path.c_str());
    ros::shutdown();
  }
  std::sort(files.begin(), files.end());

  return files;
}

// Get lines number given a file
inline int FileLinesNumber(std::ifstream &file) {
  std::string line;
  int i;
  for (i = 0; std::getline(file, line); ++i)
    ;
  file.clear();
  file.seekg(0, std::ios::beg);
  return i;
}

// Read one line for file
template <typename T>
inline bool ReadLine(std::ifstream &file, const int &data_size,
                     std::string *header, std::vector<T> *data) {
  // Read one line as std::string
  std::string line_str;
  if (!getline(file, line_str)) return false;
  // Store std::string into std::stringstream
  std::stringstream line_ss(line_str);
  // Read header
  if (header == NULL) {
    std::string temp;
    if (!getline(line_ss, temp, ',')) return false;
  } else {
    header->clear();
    if (!getline(line_ss, *header, ',')) return false;
  }
  // Read data
  std::string str;
  data->clear();
  while (getline(line_ss, str, ',')) {
    try {
      T temp(boost::lexical_cast<T>(str));
    } catch (const boost::bad_lexical_cast &e) {
      return false;
    }
    data->push_back(boost::lexical_cast<T>(str));
  }
  // Assert
  if (data->size() != data_size) {
    ROS_ERROR("[ReadLine]: Data array size %ld not equal to %d", data->size(),
              data_size);
    ros::shutdown();
  }
  return true;
}

// Clip cloud out of image
inline void ClipCameraCloud(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud, const float &min_range,
    const float &max_range, const float &min_angle, const float &max_angle) {
  for (auto pt : in_cloud->points) {
    // Remove points too close or too far away from lidar origin
    float range = sqrt(pt.x * pt.x + pt.y * pt.y);
    if (range < min_range || range > max_range) continue;
    // Remove points out of camera view
    float angle = atan2(pt.y, pt.x) * 180 / M_PI;
    if (angle < 0) angle += 360;
    if (angle < min_angle || angle > max_angle) continue;
    out_cloud->points.push_back(pt);
  }
}

// Project point cloud onto image
inline cv::Mat ProjectCloudToImage(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud,
    const cv::Mat &in_image, const Eigen::Matrix4d &extrinsic_matrix,
    const Eigen::Matrix3d &intrinsic_matrix,
    const Eigen::VectorXd &distortion_coeffs, const float &min_range,
    const float &max_range) {
  // Assert data
  if (in_cloud->empty() || in_image.empty()) {
    ROS_ERROR("[ProjectCloudToImage]: Empty cloud or image");
    ros::shutdown();
  }

  // Undistort image
  cv::Mat undistort_image;
  cv::Mat intrinsic_matrix_cv;
  eigen2cv(intrinsic_matrix, intrinsic_matrix_cv);
  cv::Mat distortion_coeffs_cv;
  eigen2cv(distortion_coeffs, distortion_coeffs_cv);
  undistort(in_image, undistort_image, intrinsic_matrix_cv,
            distortion_coeffs_cv);

  // Project cloud onto image
  // Get image size
  int image_width = undistort_image.cols;
  int image_height = undistort_image.rows;
  // Transform image form RGB space to HSV space
  cv::Mat hsv_image;
  cv::cvtColor(undistort_image, hsv_image, cv::COLOR_BGR2HSV);
  // Combine extrinsic matrix and intrinsic matrix
  Eigen::MatrixXd transform_matrix =
      intrinsic_matrix * extrinsic_matrix.topRows(3);
  // Project
  for (const auto &pt : in_cloud->points) {
    // Transform cloud from lidar coordinates to the image plane
    Eigen::Vector4d cloud_point(pt.x, pt.y, pt.z, 1);
    Eigen::Vector3d image_point = transform_matrix * cloud_point;
    image_point(0) = image_point(0) / image_point(2);
    image_point(1) = image_point(1) / image_point(2);
    // Check if image point is valid
    if ((image_point(0) >= 0 && image_point(0) < image_width) &&
        (image_point(1) >= 0 && image_point(1) < image_height)) {
      const int col = static_cast<int>(image_point(1));
      const int row = static_cast<int>(image_point(0));
      // Draw a circle in image
      float range = sqrt(pt.x * pt.x + pt.y * pt.y) - min_range;
      cv::circle(
          hsv_image, cv::Point(row, col), 5,
          cv::Scalar(static_cast<int>(range / (max_range * 2) * 255), 255, 255),
          -1);
    }
  }

  // Transform back image form HSV space to RGB space
  cv::Mat projected_image;
  cv::cvtColor(hsv_image, projected_image, cv::COLOR_HSV2BGR);

  return projected_image;
}

#endif  // LIDAR_CAMERA_CALIBRATION_UTILS_H_
