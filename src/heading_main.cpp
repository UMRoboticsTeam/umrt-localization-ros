#include <rclcpp/rclcpp.hpp>
#include "umrt-localization-ros/heading_node.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadingNode>());
    rclcpp::shutdown();
    return 0;
}