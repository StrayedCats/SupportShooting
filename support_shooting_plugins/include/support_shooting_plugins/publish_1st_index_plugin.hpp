#pragma once

#include <support_shooting_base/support_shooting_base.hpp>
#include <support_shooting_param/support_shooting_param.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace support_shooting_plugins
{
typedef vision_msgs::msg::Detection2DArray Detection2DArray;
class Publish1stIndex : public support_shooting_base::SupportShootingBase
{
public:
  void init(const support_shooting_parameters::ParamListener &) override;
  geometry_msgs::msg::PointStamped processing(
    const cv::Mat1f &,
    const vision_msgs::msg::Detection2DArray &) override;
};
}
