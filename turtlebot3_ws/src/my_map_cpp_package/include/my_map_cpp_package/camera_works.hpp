#ifndef CAMERA_WORKS_HPP
#define CAMERA_WORKS_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>

class CameraSub : public rclcpp::Node{

public:
    CameraSub();
    cv::Mat getLatestImage();

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

#endif  // CAMERA_WORKS_HPP
