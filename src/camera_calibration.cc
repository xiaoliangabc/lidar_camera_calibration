#include <ros/ros.h>

#include "lidar_camera_calibration/libCameraCalibration.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_calibration");

  ros::NodeHandle pnh("~");

  // Get data path
  std::string data_path;
  pnh.param<std::string>("data_path", data_path, "");

  // Initialize the MATLAB Compiler Runtime global state
  if (!mclInitializeApplication(NULL, 0)) {
    ROS_ERROR("Could not initialize the application properly.");
    ros::shutdown();
  }

  // Initialize the Vigenere library
  if (!libCameraCalibrationInitialize()) {
    ROS_ERROR("Could not initialize the library properly.");
    ros::shutdown();
  }

  // Camera calibration using matlab
  mwArray image_path((data_path + "image/").c_str());
  mwArray result_path((data_path + "result/").c_str());
  camera_calibration(image_path, result_path);

  // Shut down the library and the application global state.
  libCameraCalibrationTerminate();
  mclTerminateApplication();

  return 0;
}
