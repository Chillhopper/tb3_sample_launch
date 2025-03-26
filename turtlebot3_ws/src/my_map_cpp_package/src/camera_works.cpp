#include "my_map_cpp_package/camera_works.hpp"

class CameraSub : public rclcpp::Node{
public:

    CameraSub() : Node("camera_sub_node") {

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&CameraSub::imageCallback, this, std::placeholders::_1)
        );

        cv::Mat getLatestImage() {
            std::lock_guard<std::mutex> lock(image_mutex_);
            return latest_image_.clone();  // clone to avoid threading issues
        }
    
    }

private:
    cv::Mat latest_image_;
    std::mutex image_mutex_;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg){

        try{
            auto img_ptr = cv_bridge::toCvShare(msg, "bgr8");
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = img_ptr->image.clone();

            cv::imshow("view", latest_image_);
            cv::waitKey(30);
        }catch (cv_bridge::Exception &e){
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

