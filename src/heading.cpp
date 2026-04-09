//
// Created by Noah on 2024-08-18.
//

#include <rclcpp/rclcpp.hpp>
#include "umrt-localization-ros/gps_node.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsNode>());
    rclcpp::shutdown();
    return 0;
}