#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <support_shooting_base/support_shooting_base.hpp>
#include <support_shooting_param/support_shooting_param.hpp>
#include <support_shooting_plugins/publish_1st_index_plugin.hpp>
#include <rclcpp/rclcpp.hpp>

class TestPanelDetectorClass : public ::testing::Test
{
protected:
  std::unique_ptr<support_shooting_plugins::Publish1stIndex> target_class_;
  std::shared_ptr<support_shooting_parameters::ParamListener> param_listener_;

  virtual void SetUp()
  {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("test_node");

    this->target_class_ =
      std::make_unique<support_shooting_plugins::Publish1stIndex>();

    this->param_listener_ =
      std::make_shared<support_shooting_parameters::ParamListener>(
      node->get_node_parameters_interface());
    this->target_class_->init(*this->param_listener_);
  }
};

TEST_F(TestPanelDetectorClass, test_default)
{
  // 3x3 float cv::Mat
  cv::Mat1f image = cv::Mat::zeros(3, 3, CV_32FC1);
  image.at<float>(2, 1) = 3;

  vision_msgs::msg::Detection2DArray detection_msg;
  vision_msgs::msg::Detection2D detection;
  detection.bbox.center.position.x = 1;
  detection.bbox.center.position.y = 2;
  detection_msg.detections.push_back(detection);

  const auto result = this->target_class_->processing(image, detection_msg);
  EXPECT_EQ(result.point.x, 1);
  EXPECT_EQ(result.point.y, 2);
  EXPECT_EQ(result.point.z, 3);
}
