#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <support_shooting_base/support_shooting_base.hpp>
#include <support_shooting_param/support_shooting_param.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace support_shooting_node
{
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>
  SyncPolicy;
typedef sensor_msgs::msg::Image Image;
typedef vision_msgs::msg::Detection2DArray Detection2DArray;

class SupportShootingNode : public rclcpp::Node
{
public:
  SupportShootingNode(const rclcpp::NodeOptions &);

private:
  pluginlib::ClassLoader<support_shooting_base::SupportShootingBase> support_shooting_loader_;

  message_filters::Synchronizer<SyncPolicy> sync_;
  message_filters::Subscriber<Image> image_sub_;
  message_filters::Subscriber<Detection2DArray> detection_sub_;

  std::shared_ptr<support_shooting_base::SupportShootingBase> support_shoothing_;
  std::shared_ptr<support_shooting_parameters::ParamListener> param_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pose_pub_;

  void depth_callback(const Image::ConstSharedPtr, const Detection2DArray::ConstSharedPtr);
};
}
