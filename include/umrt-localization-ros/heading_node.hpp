/*

Copyright 2026, University of Manitoba Robotics Team
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
Created on Aug 3,2026 by Author: Dev Patel, Senay Yemessghen
*/

#ifndef HEADING_NODE_HPP
#define HEADING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_rel_pos_ned.hpp>
#include <string>
#include "std_msgs/msg/float32.hpp"

/**
 * @class HeadingNode
 * @brief Construct a new Heading Node:: Heading Node object. Computes robot heading from a u-blox moving-base RTK GPS pair
 * 
 * Subscribes to UBXNavRelPosNED (RELPOSNED) messages published by the starboard, which describes the relative position vector between the port and starboard antennas.
 * From that vector, this node derives the heading.
 */

class HeadingNode : public rclcpp::Node {
public:
    HeadingNode();

private:

    // Subscription that receives RELPOSNED data from starboard.
    rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>::SharedPtr relpos_sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub;

    /**
     * @brief Runs every time a new RELPOSNED message arrives.
     * @param msg The RELPOSNED message from starboard.
     */
    void relposCallback(
        const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg
    );

    /**
     * @brief Keeps an angle between 0 and 360 degrees.
     * @param heading_deg Any angle in degrees. Can be negative or over 360. 
     * @return The same angle, adjusted to fall between 0 and 360.
     */
    double normalizeHeading(double heading_deg) const;
};

#endif // HEADING_NODE_HPP