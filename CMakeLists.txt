cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(OpenCV REQUIRED)

## The catkin_package macro generates cmake config files for your package
catkin_package(
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(data_generate_node src/data_generate_node.cc)
add_executable(camera_calibration_node src/camera_calibration_node.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(data_generate_node ${catkin_LIBRARIES})
target_link_libraries(data_generate_node ${OpenCV_LIBRARIES})
target_link_libraries(camera_calibration_node ${catkin_LIBRARIES})
target_link_libraries(camera_calibration_node ${OpenCV_LIBRARIES})
