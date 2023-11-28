# SupportShooting
Support shooting function node for CoRE-1 2024

<img width="843" alt="support_shooting_overview2" src="https://github.com/StrayedCats/SupportShooting/assets/67567093/81527337-2ca7-424f-814e-6260d5305815">

## Node Structure

### Topic (Subscribe)

| Topic Name | Type | Description |
| --- | --- | --- |
| detection | vision_msgs::msg::Detection2DArray | Detected panel poses |
| depth | sensor_msgs/msg/Image | Raw Depth from camera |
| camera_info | sensor_msgs/msg/CameraInfo | Camera info for depth image |

### Topic (Publish)

| Topic Name | Type | Description |
| --- | --- | --- |
| target | geometry_msgs/msg/PointStamped | A target point for pose generation |


### Class Diagram

```mermaid
---
title: SupportShooting Plugin hierarchy
---
classDiagram
    SupportShootingNode <-- SSPluginA : load as dll
    SupportShootingNode <-- SSPluginB : load as dll
    SSPluginA <|-- SupportShootingBase : include
    SSPluginB <|-- SupportShootingBase : include
    SupportShootingNode <|-- SupportShootingBase : include
    SupportShootingNode <|-- SupportShootingCommon
    SupportShootingNode: params
    SupportShootingNode: image_callback(Image)
    SupportShootingNode: point_callback(Detection2DArray)
    class SSPluginA{
        filter_kernel_param_a
        init(params)
        processing(cv::Mat1f, geometry_msgs::msg::Detection2DArray)
    }
    class SSPluginB{
        filter_kernel_param_b
        init(params)
        processing(cv::Mat1f, geometry_msgs::msg::Detection2DArray)
    }
