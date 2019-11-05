#ifndef LIDAR_CAMERA_CALIBRATION_RVIZ_COMMAND_BUTTON_H_
#define LIDAR_CAMERA_CALIBRATION_RVIZ_COMMAND_BUTTON_H_

#include <ros/package.h>
#include <ros/ros.h>

#include <fstream>

#include <yaml-cpp/yaml.h>

#include <rviz/panel.h>

#include <QHBoxLayout>
#include <QPushButton>
#include <QSignalMapper>
#include <QVBoxLayout>

#include <std_msgs/String.h>

namespace lidar_camera_calibration {
// Declare Button subclass of rviz::Panel. Every panel which can be added via
// the Panels/Add_New_Panel menu is a subclass of rviz::Panel.
class CommandButton : public rviz::Panel {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
 public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  CommandButton(QWidget* parent = 0);

  // Declare overrides of rviz::Panel functions for saving and loading data from
  // the config file.
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  // Declare some internal slots.
 protected Q_SLOTS:
  // Call when button is clicked.
  void ButtonResponse(QString command);

  // Protected member variables.
 protected:
  // The ROS node handle.
  ros::NodeHandle nh_;

  // The ROS publisher for the command.
  ros::Publisher command_publisher_;
};  // class CommandButton

}  // namespace lidar_camera_calibration

#endif  // LIDAR_CAMERA_CALIBRATION_RVIZ_COMMAND_BUTTON_H_
