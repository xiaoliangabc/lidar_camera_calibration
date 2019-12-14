#include "lidar_camera_calibration/lidar_camera_calibration.h"

LidarCameraCalibration::LidarCameraCalibration(ros::NodeHandle nh,
                                               ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh) {
  pnh_.param<std::string>("data_path", data_path_, "");
  pnh_.param<int>("lidar_chessboard_points_number",
                  lidar_chessboard_points_number_, 0);
  pnh_.param<double>("min_angle", min_angle_, 0.0);
  pnh_.param<double>("max_angle", max_angle_, 0.0);
  pnh_.param<double>("min_range", min_range_, 0.0);
  pnh_.param<double>("max_range", max_range_, 0.0);
}

void LidarCameraCalibration::Calibration() {
  // Get camera chessboard plane model coefficients
  Eigen::MatrixXd camera_planes;
  std::vector<int> valid_indices;
  CameraPlaneModel(camera_planes, valid_indices);

  // Get lidar chessboard plane model coefficients
  Eigen::MatrixXd lidar_planes;
  LidarPlaneModel(lidar_planes, valid_indices);

  // Initial estimate of the transformation by estimating the translation
  // part and the rotation part independently
  Eigen::Matrix4d initial_trasform =
      InitialEstimate(camera_planes, lidar_planes);

  // Iterative optimization procedure with initial estimates
  Eigen::Matrix4d optimization_trasform =
      IterativeOptimization(camera_planes, initial_trasform, valid_indices);

  // Display transform matrix to console
  std::cout << "------- initial trasform -------" << std::endl;
  std::cout << initial_trasform << std::endl;
  std::cout << "------- optimizationl trasform -------" << std::endl;
  std::cout << optimization_trasform << std::endl;

  // Display calibration result by prject lidar points to image
  DisplayCalibrationResult(optimization_trasform);

  // Save calibration result to file
  SaveCalibrationResult(optimization_trasform);
}

Eigen::Matrix4d LidarCameraCalibration::InitialEstimate(
    const Eigen::MatrixXd& camera_planes, const Eigen::MatrixXd& lidar_planes) {
  Eigen::MatrixXd camera_theta = camera_planes.topRows(3);
  Eigen::MatrixXd camera_alpha = camera_planes.bottomRows(1) * -1.0;
  Eigen::MatrixXd lidar_theta = lidar_planes.topRows(3);
  Eigen::MatrixXd lidar_alpha = lidar_planes.bottomRows(1) * -1.0;

  // Compute translation vector
  Eigen::Vector3d translation_vector =
      (camera_theta * camera_theta.transpose()).inverse() * camera_theta *
      (camera_alpha - lidar_alpha).transpose();

  // Compute rotation matrix
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      lidar_theta * camera_theta.transpose(),
      Eigen::ComputeFullV | Eigen::ComputeFullU);
  Eigen::Matrix3d rotation_matrix = svd.matrixV() * svd.matrixU().transpose();
  // Optimal solution includes a reflection
  if (rotation_matrix.determinant() < 0) {
    ROS_WARN(
        "[InitialEstimate]: Initial estimate solution includes a reflection.");
    Eigen::Vector3d diag_correct;
    diag_correct << 1.0, 1.0, -1.0;
    rotation_matrix =
        svd.matrixV() * diag_correct.asDiagonal() * svd.matrixU().transpose();
  }

  return RotationMatrixToTransformAffine(rotation_matrix, translation_vector)
      .matrix();
}

Eigen::Matrix4d LidarCameraCalibration::IterativeOptimization(
    const Eigen::MatrixXd& camera_planes,
    const Eigen::Matrix4d& initial_trasform,
    const std::vector<int>& valid_indices) {
  // Combine rotaton vector and translation vector
  Eigen::Vector3d rotation_vector;
  Eigen::Vector3d translation_vector;
  TransformMatrixToRotationVector(initial_trasform, rotation_vector,
                                  translation_vector);
  Eigen::VectorXd transform(6);
  transform << rotation_vector, translation_vector;

  // Iterative optimization using Bfgs solver
  LidarCameraOptimization lidar_camera_optimization(
      data_path_, lidar_chessboard_points_number_, valid_indices,
      camera_planes);
  ROS_INFO("Initial RMS distance of points to planes: %f",
           lidar_camera_optimization(transform));
  BfgsSolver<LidarCameraOptimization> solver;
  solver.minimize(lidar_camera_optimization, transform);
  ROS_INFO("RMS distance of points to planes after search: %f",
           lidar_camera_optimization(transform));

  return RotationVectorToTransformAffine(transform.head(3), transform.tail(3))
      .matrix();
}

void LidarCameraCalibration::CameraPlaneModel(Eigen::MatrixXd& camera_planes,
                                              std::vector<int>& valid_indices) {
  //  Open camera chessboard plane model file
  std::string chessboard_model_path =
      data_path_ + "/result/camera_chessboard_model.txt";
  std::ifstream chessboard_model_file;
  chessboard_model_file.open(chessboard_model_path.c_str(), std::ios::in);
  if (!chessboard_model_file) {
    ROS_ERROR("[CameraPlaneModel]: File %s does not exist",
              chessboard_model_path.c_str());
    ros::shutdown();
  }

  // Get camera plane number size and initialize camera plane matrix
  int planes_num = FileLinesNumber(chessboard_model_file) - 1;
  if (planes_num <= 0) {
    ROS_ERROR(
        "[CameraPlaneModel]: The lines number of camera planes less than "
        "zero.");
    ros::shutdown();
  }
  camera_planes = Eigen::MatrixXd(4, planes_num);

  // Read chessboard plane model line by line
  int count = 0;
  std::string model_index;
  std::vector<double> model_data;
  // Skip file header
  ReadLine<double>(chessboard_model_file, 6, &model_index, &model_data);
  while (
      ReadLine<double>(chessboard_model_file, 6, &model_index, &model_data)) {
    // Assign rotation and translation vector
    Eigen::VectorXd transform_vector =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(model_data.data(), 6);
    Eigen::Vector3d rotation_vector = transform_vector.head(3);
    Eigen::Vector3d translation_vector = transform_vector.tail(3);
    // Transform vector to transform matrix
    Eigen::Matrix4d transform_matrix =
        RotationVectorToTransformAffine(rotation_vector, translation_vector)
            .matrix();
    // Tranform world plane model to camera plane model
    Eigen::Vector4d world_plane;
    world_plane << 0.0, 0.0, 1.0, 0.0;
    Eigen::Vector4d camera_plane =
        transform_matrix.inverse().transpose() * world_plane;
    // Assign result
    valid_indices.push_back(atoi(model_index.c_str()));
    if (camera_plane(3) > 0.0) camera_plane *= -1.0;
    camera_planes.col(count) = camera_plane;
    count++;
  }

  // Close file
  chessboard_model_file.close();
}

void LidarCameraCalibration::LidarPlaneModel(
    Eigen::MatrixXd& lidar_planes, const std::vector<int>& valid_indices) {
  // Open lidar chessboard plane model file
  std::string chessboard_model_path =
      data_path_ + "/result/lidar_chessboard_model.txt";
  std::ifstream chessboard_model_file;
  chessboard_model_file.open(chessboard_model_path.c_str(), std::ios::in);
  if (!chessboard_model_file) {
    ROS_ERROR("[LidarPlaneModel]: File %s does not exist",
              chessboard_model_path.c_str());
    ros::shutdown();
  }

  lidar_planes = Eigen::MatrixXd(4, valid_indices.size());

  // Read chessboard plane model line by line
  int count = 0;
  std::string model_index;
  std::vector<double> model_data;
  // Skip file header
  ReadLine<double>(chessboard_model_file, 4, &model_index, &model_data);
  while (
      ReadLine<double>(chessboard_model_file, 4, &model_index, &model_data)) {
    // Skip invalid index for camera detector
    if (find(valid_indices.begin(), valid_indices.end(),
             atoi(model_index.c_str())) == valid_indices.end())
      continue;
    // Assign lidar plane model
    Eigen::Vector4d lidar_plane =
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(model_data.data(), 4);
    if (lidar_plane(3) > 0.0) lidar_plane *= -1.0;
    lidar_planes.col(count) = lidar_plane;
    count++;
  }

  // Close file
  chessboard_model_file.close();
}

void LidarCameraCalibration::DisplayCalibrationResult(
    const Eigen::Matrix4d& optimization_trasform) {
  // Read camera parameters
  Eigen::MatrixXd intrinsic_matrix;
  Eigen::VectorXd distortion_coefficients;
  ReadCameraparams(intrinsic_matrix, distortion_coefficients, NULL);

  // Read point cloud data
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  std::string lidar_cloud_path = data_path_ + "cloud/";
  std::vector<std::string> lidar_cloud_files =
      GetFilesInfolder(lidar_cloud_path);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(
          lidar_cloud_path + lidar_cloud_files[0], *lidar_cloud) == -1) {
    ROS_ERROR("[DisplayCalibrationResult] Cloud reading failed.");
    ros::shutdown();
  }

  // Remove point cloud out of camera view
  pcl::PointCloud<pcl::PointXYZI>::Ptr camera_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  ClipCameraCloud(lidar_cloud, camera_cloud, min_range_, max_range_, min_angle_,
                  max_angle_);

  // Read camera image data
  std::string camera_image_path = data_path_ + "image/";
  std::vector<std::string> camera_image_files =
      GetFilesInfolder(camera_image_path);
  cv::Mat camera_image = cv::imread(camera_image_path + camera_image_files[0]);

  // Project point cloud to image
  cv::Mat camera_image_projected = ProjectCloudToImage(
      camera_cloud, camera_image, optimization_trasform, intrinsic_matrix,
      distortion_coefficients, min_range_, max_range_);

  // Display
  cv::namedWindow("lidar_camera_calibration", CV_WINDOW_NORMAL);
  cv::imshow("lidar_camera_calibration", camera_image_projected);
  cv::waitKey(0);
}

void LidarCameraCalibration::ReadCameraparams(
    Eigen::MatrixXd& intrinsic_matrix, Eigen::VectorXd& distortion_coefficients,
    cv::Size* image_size) {
  // Open camera parameters file
  std::string camera_params_path = data_path_ + "result/camera_parameters.txt";
  std::ifstream camera_params_file;
  camera_params_file.open(camera_params_path.c_str(), std::ios::in);
  if (!camera_params_file) {
    ROS_ERROR("[ReadCameraparams]: File %s does not exist",
              camera_params_path.c_str());
    ros::shutdown();
  }

  // Read camera intrinsics matrix
  std::vector<double> intrinsic_matrix_array;
  ReadLine<double>(camera_params_file, 9, NULL, &intrinsic_matrix_array);
  intrinsic_matrix =
      Eigen::Map<Eigen::Matrix<double, 3, 3> >(intrinsic_matrix_array.data())
          .transpose();

  // Read camera distortion coefficients
  std::vector<double> distortion_coefficients_array;
  ReadLine<double>(camera_params_file, 5, NULL, &distortion_coefficients_array);
  distortion_coefficients = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      distortion_coefficients_array.data(), 5);

  // Read camera image size
  if (image_size != NULL) {
    std::vector<int> image_size_array;
    ReadLine<int>(camera_params_file, 2, NULL, &image_size_array);
    image_size = new cv::Size(image_size_array[0], image_size_array[1]);
  }

  // Close camera parameters file
  camera_params_file.close();
}

void LidarCameraCalibration::SaveCalibrationResult(
    const Eigen::Matrix4d& optimization_trasform) {
  // Whether to save calibration results
  std::cout << "Whether to save calibration results(y/n): ";
  char input;
  std::cin >> input;
  std::cin.ignore(10000, '\n');
  while (input != 'y' && input != 'n') {
    std::cout << "Inupt must be 'y' or 'n', please enter again: ";
    std::cin >> input;
    std::cin.ignore(10000, '\n');
  }
  if (input == 'n') {
    ROS_INFO("[SaveCalibrationResult]: Do not save calibration result");
    return;
  }

  // Separate transform matrix to euler angles and translation vector
  Eigen::Vector3d euler_angles;
  Eigen::Vector3d translation_vector;
  TransformMatrixToEulerAngles(optimization_trasform, euler_angles,
                               translation_vector);

  // Write transform parmeters to file
  std::string result_file_path =
      data_path_ + "result/lidar_camera_parameters.txt";
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
    ROS_ERROR("[SaveCalibrationResult]: File %s does not exist",
              camera_params_path.c_str());
    ros::shutdown();
  }
  result_file << camera_params_file.rdbuf();

  // Close all files
  camera_params_file.close();
  result_file.close();

  ROS_INFO("[SaveCalibrationResult] Save calibration result to %s",
           result_file_path.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_camera_calibration_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  LidarCameraCalibration lidar_camera_calibration(nh, pnh);
  lidar_camera_calibration.Calibration();

  return 0;
}
