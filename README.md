# SupportShooting
Support shooting function node for CoRE-1 2024

<img width="811" alt="support_shoothing_overview" src="https://github.com/StrayedCats/SupportShooting/assets/67567093/7ec581ba-ec90-464a-8d85-7208a686bd56">

## Node Structure

### Topic (Subscribe)

| Topic Name | Type | Description |
| --- | --- | --- |
| positions | geometry_msgs/msg/Detection2DArray | Detected panel poses |
| depth | sensor_msgs/msg/Image | Raw Depth from camera |

### Topic (Publish)

| Topic Name | Type | Description |
| --- | --- | --- |
| target | geometry_msgs/msg/PointStamped | A target point for pose generation |


### Class Diagram

```mermaid
---
title: Panel Detector Plugin hierarchy
---
classDiagram
    SupportShootingNode <-- SSPluginA : load as dll
    SupportShootingNode <-- SSPluginB : load as dll
    SSPluginA <|-- DetectorBase : include
    SSPluginB <|-- DetectorBase : include
    SupportShootingNode <|-- DetectorBase : include
    SupportShootingNode: params
    SupportShootingNode: image_callback(Image)
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
