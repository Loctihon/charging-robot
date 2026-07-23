#pragma once

#include <atomic>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

namespace ev_charging_port_gazebo_plugins
{

// Sensor plugin attached to the charging-port cover's contact sensor.
// On the first registered contact (or on a debug trigger, for testing
// without physics) it hides the cover's visual (simulating the flap
// opening) and publishes the event on a ROS 2 topic.
class CoverOpenPlugin : public gazebo::SensorPlugin
{
public:
  CoverOpenPlugin() = default;
  ~CoverOpenPlugin() override = default;

  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  // Gazebo 11's ContactSensor only exposes the base no-arg "updated" signal;
  // the current contacts are read back via Contacts() from inside it.
  void OnSensorUpdate();

  // Hides the cover visual and publishes cover_state=true. Shared by the
  // real contact-sensor path and the debug_trigger_topic_ test hook; the
  // triggered_ guard makes it safe to call from either, more than once.
  void TriggerOpen();

  gazebo::sensors::ContactSensorPtr contact_sensor_;
  gazebo::event::ConnectionPtr update_connection_;

  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::PublisherPtr visual_pub_;

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cover_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr debug_trigger_sub_;

  std::string visual_scoped_name_;
  std::string visual_parent_name_;
  std::atomic_bool triggered_{false};
};

}  // namespace ev_charging_port_gazebo_plugins
