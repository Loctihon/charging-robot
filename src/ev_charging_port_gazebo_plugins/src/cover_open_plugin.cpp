#include "ev_charging_port_gazebo_plugins/cover_open_plugin.hpp"

namespace ev_charging_port_gazebo_plugins
{

namespace
{
// "a::b::c" -> "a::b". gazebo::msgs::Visual requires parent_name to be set
// or the message fails IsInitialized() and Gazebo silently drops it.
std::string ParentScopedName(const std::string & scoped_name)
{
  const auto pos = scoped_name.rfind("::");
  return pos == std::string::npos ? scoped_name : scoped_name.substr(0, pos);
}
}  // namespace

void CoverOpenPlugin::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  contact_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(sensor);
  if (!contact_sensor_) {
    gzerr << "CoverOpenPlugin requires a <sensor type=\"contact\">, got ["
          << sensor->Type() << "] instead.\n";
    return;
  }

  visual_scoped_name_ = sdf->Get<std::string>(
    "visual_scoped_name", "ev_charging_port::cover::cover_visual").first;
  visual_parent_name_ = sdf->Get<std::string>(
    "visual_parent_name", ParentScopedName(visual_scoped_name_)).first;
  const std::string ros_topic = sdf->Get<std::string>(
    "ros_topic", "/charging_port/cover_state").first;
  const std::string debug_trigger_topic = sdf->Get<std::string>(
    "debug_trigger_topic", "/charging_port/debug_trigger_cover").first;

  // Gazebo-transport publisher used to hide the cover's visual in gzclient.
  gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gz_node_->Init(contact_sensor_->WorldName());
  visual_pub_ = gz_node_->Advertise<gazebo::msgs::Visual>("~/visual");

  // ROS 2 side, managed by gazebo_ros so it shares the plugin executor.
  ros_node_ = gazebo_ros::Node::Get(sdf);
  cover_state_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
    ros_topic, rclcpp::QoS(1).transient_local());

  // Testing hook: trigger the open behavior directly, no physics/contact
  // required, e.g. `ros2 topic pub -1 /charging_port/debug_trigger_cover
  // std_msgs/msg/Empty "{}"`.
  debug_trigger_sub_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
    debug_trigger_topic, rclcpp::QoS(1),
    [this](std_msgs::msg::Empty::ConstSharedPtr) {TriggerOpen();});

  update_connection_ = contact_sensor_->ConnectUpdated(
    std::bind(&CoverOpenPlugin::OnSensorUpdate, this));
  contact_sensor_->SetActive(true);

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "CoverOpenPlugin watching sensor [%s], will publish to [%s] on first contact "
    "(or immediately on [%s] for testing)",
    contact_sensor_->Name().c_str(), ros_topic.c_str(), debug_trigger_topic.c_str());
}

void CoverOpenPlugin::OnSensorUpdate()
{
  const gazebo::msgs::Contacts contacts = contact_sensor_->Contacts();
  if (contacts.contact_size() == 0) {
    return;
  }
  TriggerOpen();
}

void CoverOpenPlugin::TriggerOpen()
{
  if (triggered_.exchange(true)) {
    return;
  }

  gazebo::msgs::Visual visual_msg;
  visual_msg.set_name(visual_scoped_name_);
  visual_msg.set_parent_name(visual_parent_name_);
  visual_msg.set_visible(false);
  visual_pub_->Publish(visual_msg);

  std_msgs::msg::Bool state_msg;
  state_msg.data = true;
  cover_state_pub_->publish(state_msg);

  RCLCPP_INFO(ros_node_->get_logger(), "Charging port cover opened.");
}

}  // namespace ev_charging_port_gazebo_plugins

GZ_REGISTER_SENSOR_PLUGIN(ev_charging_port_gazebo_plugins::CoverOpenPlugin)
