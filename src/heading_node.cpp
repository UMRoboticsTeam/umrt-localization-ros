/*

Copyright 2026, University of Manitoba Robotics Team
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
Created on Aug 3,2026 by Author: Dev Patel, Senay Yemessghen
*/

#include "umrt-localization-ros/heading_node.hpp"
#include <ublox_ubx_msgs/msg/carr_soln.hpp>

#include <cmath>
#include <string>

/**
 * @brief Constructor. Sets up the subscription to the starboard.
 */

HeadingNode::HeadingNode() : Node("heading_node")
{
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.transient_local();

    const std::string relpos_topic = this->declare_parameter<std::string>("relpos_topic", "/gps_starboard/ubx_nav_rel_pos_ned");
    relpos_sub = this->create_subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>(relpos_topic, qos, [this](const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg) {relposCallback(msg);});
    
    RCLCPP_INFO(this->get_logger(), "Moving-base RTK heading node initialized.");
}


/**
 * @brief Keeps an angle between 0 and 360 degrees.
 * @param heading_deg Any angle in degrees. Can be negative or over 360. 
 * @return The same angle, adjusted to fall between 0 and 360.
 */
double HeadingNode::normalizeHeading(double heading_deg) const
{
    while (heading_deg < 0.0) {
        heading_deg += 360.0;
    }
    while (heading_deg >= 360.0) {
        heading_deg -= 360.0;
    }
    return heading_deg;
}


/**
 * @brief Turns the RELPOSNED message into a heading and outputs it.
 *
 * RELPOSNED gives us the position of the starboard antenna relative to the port antenna. 
 * On the rover, port is the left antenna and starboard is the right antenna, so this vector points sideways across the rover.
 * The rover's actual forward heading is 90 degrees off from that sideways vector.
 *
 * Before calculating the heading, we check three things:
 *   1. The GPS fix is valid.
 *   2. RTK has a full, fixed solution (not just a rough float estimate).
 *   3. The measured distance between antennas matches what we expect - if it doesn't, something went wrong with the RTK fix.
 * If any check fails, we log a warning and skip that message.
 *
 * @param msg The RELPOSNED message from the starboard.
 */
void HeadingNode::relposCallback(const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg)
{
    // Check 1: Is the GPS fix actually valid
    if (!msg->gnss_fix_ok || !msg->rel_pos_valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "RELPOSNED not valid yet (gnss_fix_ok=%d, rel_pos_valid=%d); skipping.", static_cast<int>(msg->gnss_fix_ok), static_cast<int>(msg->rel_pos_valid));
        return;
    }

    // Check 2: carr_soln tells us how good the RTK fix is.
    // 0 = no fix, 1 = rough (float) fix, 2 = precise (fixed) solution.
    // We only move ahead if there is a fixed solution since a float fix can be several degrees off.
    const uint8_t carr_soln = msg->carr_soln.status;
    if (carr_soln != ublox_ubx_msgs::msg::CarrSoln::CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "RTK not fixed (carr_soln=%d); heading unreliable, skipping.", static_cast<int>(carr_soln));
        return;
    }

    // Convert the raw position values (cm + extra high-precision digits) into meters.
    const double rel_n =
        static_cast<double>(msg->rel_pos_n) * 1e-2 +
        static_cast<double>(msg->rel_pos_hp_n) * 1e-4;
    const double rel_e =
        static_cast<double>(msg->rel_pos_e) * 1e-2 +
        static_cast<double>(msg->rel_pos_hp_e) * 1e-4;

    if (rel_n == 0.0 && rel_e == 0.0) {
        RCLCPP_WARN(this->get_logger(), "RELPOSNED baseline is zero; cannot compute heading.");
        return;
    }

    // Check 3: Check if the distance between antennas match what we expect
    // kExpectedBaselineM should be the real, measured port-to-starboard antenna distance in meters for this rover (update it if the antennas move).
    // If the measured distance is off, the RTK fix is probably wrong.
    constexpr double kExpectedBaselineM = 0.43;
    constexpr double kBaselineToleranceM = 0.05;  // +/- 5 cm

    const double baseline_len = std::sqrt(rel_n * rel_n + rel_e * rel_e);
    if (std::fabs(baseline_len - kExpectedBaselineM) > kBaselineToleranceM) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Baseline length %.3f m differs from expected %.2f m; possible wrong ambiguity fix, skipping.", baseline_len, kExpectedBaselineM);
        return;
    }

    // Calculate the heading. baseline_heading is the direction from port to starboard (sideways). Subtracting 90 degrees turns that into the rover's actual forward-facing heading.
    double baseline_heading =
        std::atan2(rel_e, rel_n) * 180.0 / M_PI;
    baseline_heading = normalizeHeading(baseline_heading);

    const double heading_angle = normalizeHeading(baseline_heading - 90.0);

    // Turn the numeric heading into a compass direction (N, NE, E, ...)
    std::string heading_dir = "N/A";
    if (heading_angle >= 337.5 || heading_angle < 22.5)  heading_dir = "N";
    else if (heading_angle < 67.5)   heading_dir = "NE";
    else if (heading_angle < 112.5)  heading_dir = "E";
    else if (heading_angle < 157.5)  heading_dir = "SE";
    else if (heading_angle < 202.5)  heading_dir = "S";
    else if (heading_angle < 247.5)  heading_dir = "SW";
    else if (heading_angle < 292.5)  heading_dir = "W";
    else if (heading_angle < 337.5)  heading_dir = "NW";

    RCLCPP_INFO(this->get_logger(), "Heading: %s, angle: %.2f deg, baseline heading: %.2f deg, rel_n: %.3f m, rel_e: %.3f m, len: %.3f m", heading_dir.c_str(), heading_angle, baseline_heading, rel_n, rel_e, baseline_len);
}