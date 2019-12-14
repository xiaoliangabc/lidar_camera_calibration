#include <ros/ros.h>

#include "lidar_camera_calibration/libCameraCalibration.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_calibration");

  ros::NodeHandle pnh("~");

  // Get data path
  std::string data_path;
  pnh.param<std::string>("common/data_path", data_path, "");
  int square_size;
  pnh.param<int>("common/square_size", square_size, 100);

  // Initialize the MATLAB Compiler Runtime global state
  ROS_INFO("Start initialize the MATLAB  Compiler Runtime global state.");
  if (!mclInitializeApplication(NULL, 0)) {
    ROS_ERROR("Could not initialize the application properly.");
    ros::shutdown();
  }
  ROS_INFO("Finish initialize the MATLAB  Compiler Runtime global state.");

  // Initialize the CameraCalibration library
  ROS_INFO("Start initialize the CameraCalibration library.");
  if (!libCameraCalibrationInitialize()) {
    ROS_ERROR("Could not initialize the library properly.");
    ros::shutdown();
  }
  ROS_INFO("Finish initialize the CameraCalibration library.");

  // Camera calibration using matlab
  ROS_INFO("Start camera calibration.");
  mwArray image_path((data_path + "image/").c_str());
  mwArray result_path((data_path + "result/").c_str());
  mwArray square_size_mw(square_size);
  camera_calibration(image_path, result_path, square_size_mw);
  ROS_INFO("Finish camera calibration.");

  // Shut down the library and the application global state.
  libCameraCalibrationTerminate();
  mclTerminateApplication();
  ROS_INFO(
      "Shut down the library and the application global state, exiting...");

  return 0;
}
