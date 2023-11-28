#include "support_shooting_common/depth_processing.hpp"

namespace support_shooting_common
{

DepthProcessing::DepthProcessing()
{
}

DepthProcessing::~DepthProcessing()
{
}

void DepthProcessing::setParams(const float fx, const float fy, const float cx, const float cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
}

Detection3DArray DepthProcessing::convertDetTo3D(
  const cv::Mat1f & depth_image,
  const Detection2DArray & detection_2d_array)
{
  if (fx_ == 0.0f || fy_ == 0.0f || cx_ == 0.0f || cy_ == 0.0f) {
    std::cerr << "Error: DepthProcessing::convertDetTo3D(): fx, fy, cx, cy are not set." <<
      std::endl;
    return Detection3DArray();
  }
  if (depth_image.empty()) {
    std::cerr << "Error: DepthProcessing::convertDetTo3D(): depth_image is empty." << std::endl;
    return Detection3DArray();
  }

  Detection3DArray detection_3d_array;
  detection_3d_array.header = detection_2d_array.header;
  detection_3d_array.detections.resize(detection_2d_array.detections.size());

  for (size_t i = 0; i < detection_2d_array.detections.size(); ++i) {
    const auto & detection_2d = detection_2d_array.detections[i];
    auto & detection_3d = detection_3d_array.detections[i];

    detection_3d.header = detection_2d.header;
    detection_3d.results.resize(detection_2d.results.size());

    // detection_2d : x, y, width, height only

    for (size_t j = 0; j < detection_2d.results.size(); ++j) {
      const auto & detection_2d_result = detection_2d.results[j];
      auto & detection_3d_result = detection_3d.results[j];

      detection_3d_result.hypothesis = detection_2d_result.hypothesis;
      detection_3d_result.pose = detection_2d_result.pose;

      const auto x = detection_2d_array.detections[i].bbox.center.position.x;
      const auto y = detection_2d_array.detections[i].bbox.center.position.y;
      const auto width = detection_2d_array.detections[i].bbox.size_x;
      const auto height = detection_2d_array.detections[i].bbox.size_y;

      const auto z = static_cast<float>(depth_image.at<float>(y + height / 2, x + width / 2));
      const auto x_ = (x + width / 2 - cx_) * z / fx_;
      const auto y_ = (y + height / 2 - cy_) * z / fy_;

      detection_3d_array.detections[i].bbox.center.position.x = x_;
      detection_3d_array.detections[i].bbox.center.position.y = y_;
      detection_3d_array.detections[i].bbox.center.position.z = z;

    }
  }

  return detection_3d_array;
}

} // namespace support_shooting_common
