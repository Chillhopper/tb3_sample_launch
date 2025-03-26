#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "my_map_cpp_package/camera_works.hpp"

// camera topic -> /camera/image_raw






class Wander : public BT::SyncActionNode{
    public:
        
        Wander(const std::string& name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node) : BT::SyncActionNode(name, config), node_(node){

            publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        }

        static BT::PortsList providedPorts()
        {
            return {};
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {
            std::cout << "Wandering " << this->name() << std::endl;

            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.2;  // Move forward
            msg.angular.z = 0.0; // No rotation
    
            publisher_->publish(msg);

            return BT::NodeStatus::SUCCESS;
        }
    
        private:

            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

};

class MoveTowardsCube : public BT::SyncActionNode
{
public:
    MoveTowardsCube(const std::string &name, const BT::NodeConfiguration &config,
                    std::shared_ptr<CameraSub> camera_node,
                    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub)
        : BT::SyncActionNode(name, config), camera_node_(camera_node), cmd_pub_(cmd_pub) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override
    {
        cv::Mat image = camera_node_->getLatestImage();

        if (image.empty()) {
            RCLCPP_WARN(camera_node_->get_logger(), "No image received yet.");
            return BT::NodeStatus::FAILURE;
        }

        // Convert to HSV
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        // Red color range
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
        cv::Mat red_mask = mask1 | mask2;

        int red_pixels = cv::countNonZero(red_mask);
        RCLCPP_INFO(camera_node_->get_logger(), "Red pixel count: %d", red_pixels);

        if (red_pixels > 1000)
        {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.2;
            msg.angular.z = 0.0;
            cmd_pub_->publish(msg);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    std::shared_ptr<CameraSub> camera_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};


// class CheckCube : public BT::SyncActionNode{
//     public:

//         CheckCube(const std::string &name, const BT::NodeConfiguration &config,
//             std::shared_ptr<CameraSub> camera_node) : BT::SyncActionNode(name, config) {
            
//         }

//         static BT::PortsList providedPorts()
//         {
//             return {};
//         }

//         BT::NodeStatus tick() override
//         {
//             std::cout << "CheckCube: " << this->name() << std::endl;
//             return BT::NodeStatus::SUCCESS;
//         }

// }

static const char* xml_text_medium = R"(

<root main_tree_to_execute = "MainTree" >

    <BehaviorTree ID="MainTree">
       <Fallback name="root">
           <Task1 name="task_1"/>
           <Task2 name="task_2"/>
           <Sequence>
               <Task3 name="task_3"/>
               <Task4  name="task_4"/>
           </Sequence>
       </Fallback>
    </BehaviorTree>

</root>
)";



class NavMain : public rclcpp::Node {
    public:
        NavMain() : Node("nav_main_node") {
            RCLCPP_INFO(this->get_logger(), "Navigation Node Initialized");
        }

    private:

};


int main(int argc, char **argv) {
    
    rclcpp::init(argc, argv);


    rclcpp::spin(std::make_shared<NavMain>());
    rclcpp::shutdown();
    return 0;

}

