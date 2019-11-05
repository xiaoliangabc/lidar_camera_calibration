#include "lidar_camera_calibration/rviz_command_button.h"

namespace lidar_camera_calibration {
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
CommandButton::CommandButton(QWidget* parent) : rviz::Panel(parent) {
  // Get button parameters
  std::string current_path = ros::package::getPath("lidar_camera_calibration");
  std::string button_layout_file = current_path + "/param/button_layout.yaml";
  YAML::Node node = YAML::LoadFile(button_layout_file);
  std::vector<std::string> button_names;
  for (int i = 0; i < node.size(); ++i) {
    button_names.push_back(node[i]["name"].as<std::string>());
  }

  // Lay out buttons using QPushButton in a QHBoxLayout.
  QHBoxLayout* layout = new QHBoxLayout;
  QSignalMapper* signal_mapper = new QSignalMapper(this);
  for (auto button_name : button_names) {
    QPushButton* button = new QPushButton(QString::fromStdString(button_name));
    layout->addWidget(button);
    button->setEnabled(true);
    connect(button, SIGNAL(clicked()), signal_mapper, SLOT(map()));
    signal_mapper->setMapping(button, QString::fromStdString(button_name));
  }
  connect(signal_mapper, SIGNAL(mapped(QString)), this,
          SLOT(ButtonResponse(QString)));
  setLayout(layout);

  // Publisher
  command_publisher_ = nh_.advertise<std_msgs::String>("/button_command", 1);
}

void CommandButton::ButtonResponse(QString command) {
  std_msgs::String command_msg;
  command_msg.data = command.toStdString();
  command_publisher_.publish(command_msg);
}

// Save all configuration data from this panel to the given Config object.
void CommandButton::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void CommandButton::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

}  // lidar_camera_calibration

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lidar_camera_calibration::CommandButton, rviz::Panel)
