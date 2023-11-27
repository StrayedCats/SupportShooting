#include "support_shooting_plugins/publish_1st_index_plugin.hpp"

namespace support_shooting_plugins
{

void Publish1stIndex::init(const support_shooting_parameters::ParamListener & param_listener)
{
  (void)param_listener;
}
geometry_msgs::msg::PointStamped Publish1stIndex::processing(
  const cv::Mat1f & depth,
  const vision_msgs::msg::Detection2DArray & detection)
{
  // get depth point from 1st index
  const vision_msgs::msg::Detection2D & detection2d = detection.detections[0];
  const vision_msgs::msg::BoundingBox2D & bbox = detection2d.bbox;
  const int x = (int)bbox.center.position.x;
  const int y = (int)bbox.center.position.y;
  const float z = depth.at<float>(y, x);

  geometry_msgs::msg::PointStamped result;
  result.header = detection.header;
  result.point.x = x;
  result.point.y = y;
  result.point.z = z;
  return result;
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  support_shooting_plugins::Publish1stIndex,
  support_shooting_base::SupportShootingBase)
