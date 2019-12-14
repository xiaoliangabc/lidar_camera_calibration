#include "lidar_camera_calibration/lidar_camera_optimization.h"

LidarCameraOptimization::LidarCameraOptimization(
    std::string data_path, int lidar_chessboard_points_number,
    std::vector<int> valid_indices, Eigen::MatrixXd camera_planes) {
  // Open lidar chessboard plane points file
  std::string lidar_chessboard_points_path =
      data_path + "result/lidar_chessboard_points.txt";
  std::ifstream lidar_chessboard_points_file;
  lidar_chessboard_points_file.open(lidar_chessboard_points_path.c_str(),
                                    std::ios::in);
  if (!lidar_chessboard_points_file) {
    ROS_ERROR("[LidarCameraOptimization]: File %s does not exist",
              lidar_chessboard_points_path.c_str());
    ros::shutdown();
  }

  // Read chessboard plane points line by line
  lidar_chessboard_points_.resize(valid_indices.size());
  Eigen::MatrixXd sigle_chessboard_poins =
      Eigen::MatrixXd(3, lidar_chessboard_points_number);
  int plane_count = 0;
  int points_count = 0;
  std::string points_index;
  std::vector<double> points_data;
  // Skip file header
  ReadLine<double>(lidar_chessboard_points_file, 3, &points_index,
                   &points_data);
  while (ReadLine<double>(lidar_chessboard_points_file, 3, &points_index,
                          &points_data)) {
    // Skip invalid index for camera detector
    if (find(valid_indices.begin(), valid_indices.end(),
             atoi(points_index.c_str())) == valid_indices.end())
      continue;
    // Assign lidar chessboard plane points
    Eigen::Vector3d point =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(points_data.data(), 3);
    sigle_chessboard_poins.col(points_count) = point;
    if (points_count == lidar_chessboard_points_number - 1) {
      lidar_chessboard_points_[plane_count] = sigle_chessboard_poins;
      points_count = -1;
      plane_count++;
    }
    points_count++;
  }

  // Parse camera chessboard plane model
  camera_theta_ = camera_planes.topRows(3);
  camera_alpha_ = camera_planes.bottomRows(1) * -1.0;
}

double LidarCameraOptimization::value(const Eigen::VectorXd &tranform) {
  Eigen::Matrix3d rotation_matrix =
      RotationVectorToRotationMatrix(tranform.head(3));
  Eigen::Vector3d translation_vector = tranform.tail(3);
  // Compute RSM error in distance to planes
  double count = 0.0;
  for (int i = 0; i < lidar_chessboard_points_.size(); ++i) {
    Eigen::MatrixXd camera_theta = camera_theta_.col(i).transpose();
    Eigen::MatrixXd translation_matrix =
        translation_vector.replicate(1, lidar_chessboard_points_[i].cols());
    Eigen::MatrixXd camera_alpha_matrix =
        camera_alpha_.col(i).replicate(1, lidar_chessboard_points_[i].cols());
    Eigen::MatrixXd diff =
        camera_theta * (rotation_matrix * lidar_chessboard_points_[i] +
                        translation_matrix) -
        camera_alpha_matrix;
    count += diff.array().pow(2).mean();
  }
  return sqrt(count / lidar_chessboard_points_.size());
}
