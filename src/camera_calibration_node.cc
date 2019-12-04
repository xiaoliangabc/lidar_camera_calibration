#include "lidar_camera_calibration/camera_calibration_node.h"

CameraCalibrationNode::CameraCalibrationNode() : nh_("~") {
  nh_.param<std::string>("file_path", file_path_, "");
  nh_.param<int>("pattern_width", pattern_width_, 6);
  nh_.param<int>("pattern_height", pattern_height_, 8);
  nh_.param<int>("square_size", square_size_, 10);
}

void CameraCalibrationNode::Calibration() {
  // Get all image files
  std::string image_path = file_path_ + "image/";
  std::vector<std::string> image_files = get_files(image_path);
  ROS_INFO("[DataGenerateNode::Calibration] Get %ld images",
           image_files.size());

  // Interior number of corners
  cv::Size pattern_size(pattern_width_, pattern_height_);
  // Image size
  cv::Size image_size;
  // 2D points in image plane
  std::vector<std::vector<cv::Point2f>> image_points;
  for (auto image_flie : image_files) {
    ROS_INFO("[DataGenerateNode::Calibration] Processing %s",
             image_flie.c_str());

    // Read origin image
    cv::Mat image = cv::imread(image_path + image_flie);
    image_size = image.size();

    // Covert color image to gray image
    cv::Mat gray_image;
    cvtColor(image, gray_image, CV_BGR2GRAY);

    // Find the chessboard corners
    std::vector<cv::Point2f> corners;
    bool pattern_found =
        findChessboardCorners(gray_image, pattern_size, corners);

    if (pattern_found) {
      ROS_INFO("[DataGenerateNode::Calibration] Find pattern corners");

      // Find sub pixel corners
      find4QuadCornerSubpix(gray_image, corners, cv::Size(11, 11));

      // Put all image corners to a vector
      image_points.push_back(corners);

      // Draw corners in image
      drawChessboardCorners(image, pattern_size, corners, pattern_found);

      // Show image with corners
      cv::imshow("camera_calibration", image);
      cv::waitKey(0);
    } else {
      ROS_INFO("[DataGenerateNode::Calibration] Can not find pattern corners");
      exit(EXIT_FAILURE);
    }
  }

  // Generate 3D object points in real world space for all images
  std::vector<std::vector<cv::Point3f>> object_points;
  // Square size of real Chessboard
  cv::Size square_size = cv::Size(square_size_, square_size_);
  for (int k = 0; k < image_points.size(); k++) {
    // 3D object points for one image
    std::vector<cv::Point3f> points;
    for (int i = 0; i < pattern_size.height; i++)
      for (int j = 0; j < pattern_size.width; j++) {
        cv::Point3f point;
        point.x = i * square_size.width;
        point.y = j * square_size.height;
        point.z = 0;  // z coordinate must be 0 (homography)
        points.push_back(point);
      }
    object_points.push_back(points);
  }
  ROS_INFO("[DataGenerateNode::Calibration] Generate 3D object points");

  // Camera intrinsic matrix
  cv::Mat intrinsic_matrix;
  // Camera distortion coefficients
  cv::Mat distortion_coeffs;
  // Camera extrinsic rotation vectors
  std::vector<cv::Mat> rotation_vectors;
  // Camera extrinsic translation vectors
  std::vector<cv::Mat> translation_vectors;
  // Calibrate camera
  calibrateCamera(object_points, image_points, image_size, intrinsic_matrix,
                  distortion_coeffs, rotation_vectors, translation_vectors);
  ROS_INFO("[DataGenerateNode::Calibration] Calibration finish");

  // Calculate calibration error
  float error = 0.0;
  for (int i = 0; i < object_points.size(); ++i) {
    // Transform rotation vector to matrix
    cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    Rodrigues(rotation_vectors[i], rotation_matrix);

    // Project 3D object points to image plane
    std::vector<cv::Point2f> project_corners;
    projectPoints(object_points[i], rotation_vectors[i], translation_vectors[i],
                  intrinsic_matrix, distortion_coeffs, project_corners);

    // Calculate error between project corners and detected corners
    error += norm(image_points[i], project_corners, cv::NORM_L2);
  }
  error /= (object_points.size() * object_points[0].size());
  ROS_INFO("[DataGenerateNode::Calibration] Calibration error: %f", error);

  // Write calibration result
  std::string calibration_result =
      file_path_ + "result/" + "camera_calibration.txt";
  std::ofstream fout(calibration_result);
  fout << "intrinsic matrix: " << std::endl;
  fout << intrinsic_matrix << std::endl;
  fout << "distortion coefficients: " << std::endl;
  fout << distortion_coeffs << std::endl;
  fout.close();
  ROS_INFO("[DataGenerateNode::Calibration] Save calibration result to %s",
           calibration_result.c_str());
}

std::vector<std::string> CameraCalibrationNode::get_files(std::string path) {
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
    ROS_ERROR("[DataGenerateNode::get_files] Could not open directory %s",
              path.c_str());
    exit(EXIT_FAILURE);
  }
  std::sort(files.begin(), files.end());

  return files;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_calibration_node");

  CameraCalibrationNode camera_calibration_node;
  camera_calibration_node.Calibration();

  ros::spinOnce();

  return 0;
}
