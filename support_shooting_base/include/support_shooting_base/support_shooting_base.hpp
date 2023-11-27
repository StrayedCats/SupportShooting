#pragma once

#include <geometry_msgs/msg/point_stamped.hpp>
#include <support_shooting_param/support_shooting_param.hpp>
#include <vector>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <opencv2/opencv.hpp>

namespace support_shooting_base
{
class SupportShootingBase
{
public:
  virtual void init(const support_shooting_parameters::ParamListener &) = 0;
  virtual geometry_msgs::msg::PointStamped processing(
    const cv::Mat1f &,
    const vision_msgs::msg::Detection2DArray &) =
  0;
  virtual ~SupportShootingBase() {}

protected:
  SupportShootingBase() {}
};
}
