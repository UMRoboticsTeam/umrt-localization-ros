/*

Copyright 2026, University of Manitoba Robotics Team
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
Created on Aug 3,2026 by Author: Dev Patel, Senay Yemessghen
*/

#include <rclcpp/rclcpp.hpp>
#include "umrt-localization-ros/heading_node.hpp"


/**
 * @brief Starts up ROS, runs HeadingNode until shutdown, then exits cleanly.
 */
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadingNode>());
    rclcpp::shutdown();
    return 0;
}