cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rviz
  sensor_msgs
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/ManualCalibration.cfg
  cfg/ChessboardDetector.cfg
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  LIBRARIES lidar_camera_calibration
  CATKIN_DEPENDS roscpp dynamic_reconfigure
)

## MATLAB runtime dir
set(MATLAB_DIR /usr/local/MATLAB/R2019a)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MATLAB_DIR}/extern/include/
)

## Declare a C++ executable
add_executable(data_generate src/data_generate.cc)
add_executable(camera_calibration src/camera_calibration.cc)
add_executable(lidar_chessboard_detector src/lidar_chessboard_detector.cc)
add_executable(lidar_camera_calibration_node src/lidar_camera_calibration.cc src/lidar_camera_optimization)
add_executable(manual_calibration src/manual_calibration.cc)

## Add cmake target dependencies of the library
add_dependencies(lidar_chessboard_detector ${PROJECT_NAME}_gencfg)
add_dependencies(manual_calibration ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(data_generate ${catkin_LIBRARIES})
target_link_libraries(data_generate ${OpenCV_LIBRARIES})
target_link_libraries(camera_calibration ${catkin_LIBRARIES})
target_link_libraries(camera_calibration ${PROJECT_SOURCE_DIR}/lib/libCameraCalibration.so)
target_link_libraries(camera_calibration ${MATLAB_DIR}/sys/os/glnxa64/libstdc++.so.6)
target_link_libraries(camera_calibration ${MATLAB_DIR}/runtime/glnxa64/libmwmclmcrrt.so)
target_link_libraries(camera_calibration ${MATLAB_DIR}/bin/glnxa64/libQt5Core.so.5)
target_link_libraries(lidar_chessboard_detector ${catkin_LIBRARIES})
target_link_libraries(lidar_camera_calibration_node ${catkin_LIBRARIES})
target_link_libraries(lidar_camera_calibration_node ${OpenCV_LIBRARIES})
target_link_libraries(manual_calibration ${catkin_LIBRARIES})
target_link_libraries(manual_calibration ${OpenCV_LIBRARIES})

## This setting causes Qt's "MOC" generation to happen automatically
set(CMAKE_AUTOMOC ON)

## Use the Qt version that rviz used so they are compatible
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## Define QT_NO_KEYWORDS, avoid defining "emit", "slots", etc
add_definitions(-DQT_NO_KEYWORDS)

## Specify the list of source files
set(SRC_FILES
  src/rviz_command_button.cc
  ${QT_MOC}
)

## Specify the list of header files
set(HEADER_FILES
   include/lidar_camera_calibration/rviz_command_button.h
)

## Declare library
add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

## Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  rviz_command_button.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
