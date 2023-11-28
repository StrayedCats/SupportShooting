#pragma once

#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace support_shooting_common
{
using Point = geometry_msgs::msg::Point;
using Detection2DArray = vision_msgs::msg::Detection2DArray;
using Detection3DArray = vision_msgs::msg::Detection3DArray;

class DepthProcessing
{
public:
  explicit DepthProcessing();
  ~DepthProcessing();

  void setParams(const float, const float, const float, const float);
  Detection3DArray convertDetTo3D(const cv::Mat1f &, const Detection2DArray &);

private:
  float fx_;
  float fy_;
  float cx_;
  float cy_;
};

} // namespace support_shooting_common
