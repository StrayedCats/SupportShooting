#include <support_shooting_node/support_shooting_node.hpp>

namespace support_shooting_node
{

SupportShootingNode::SupportShootingNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("support_shooting_node", options),
  support_shooting_loader_("support_shooting_base", "support_shooting_base::SupportShootingBase"),
  sync_(1)
{
  this->param_listener_ = std::make_shared<support_shooting_parameters::ParamListener>(
    this->get_node_parameters_interface());
  const auto params = this->param_listener_->get_params();

  try {
    this->support_shoothing_ = this->support_shooting_loader_.createSharedInstance(
      params.load_target_plugin);
    this->support_shoothing_->init(*this->param_listener_);
    std::cout << "params.load_target_plugin: " << params.load_target_plugin << std::endl;
  } catch (pluginlib::PluginlibException & ex) {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  this->image_sub_.subscribe(this, "depth", rmw_qos_profile_sensor_data);
  this->detection_sub_.subscribe(this, "detection", rmw_qos_profile_sensor_data);
  this->sync_.connectInput(this->image_sub_, this->detection_sub_);

  this->pose_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("pose", 1);

  using namespace std::placeholders;  // NOLINT
  this->sync_.registerCallback(
    std::bind(&SupportShootingNode::depth_callback, this, _1, _2));

}

void SupportShootingNode::depth_callback(
  const Image::ConstSharedPtr image_msg,
  const Detection2DArray::ConstSharedPtr detection_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const auto result = this->support_shoothing_->processing(cv_ptr->image, *detection_msg);
  this->pose_pub_->publish(result);
}

}  // namespace support_shooting_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(support_shooting_node::SupportShootingNode)
