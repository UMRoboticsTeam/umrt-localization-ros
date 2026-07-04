#include "umrt-localization-ros/gps_node.hpp"

#include <cmath>
#include <string>

GpsNode::GpsNode() : Node("gps_node")
{
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.transient_local();

    relpos_sub = this->create_subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>(
        "/rover/ubx_nav_rel_pos_ned",
        qos,
        std::bind(&GpsNode::relposCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Moving-base RTK heading node initialized.");
}

double GpsNode::normalizeHeading(double heading_deg) const
{
    while (heading_deg < 0.0) {
        heading_deg += 360.0;
    }

    while (heading_deg >= 360.0) {
        heading_deg -= 360.0;
    }

    return heading_deg;
}

void GpsNode::relposCallback(
    const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg
)
{
    /*
     * RELPOSNED gives the vector from the moving base antenna
     * to the rover antenna.
     *
     * Your setup:
     *   left antenna  = base
     *   right antenna = rover
     *
     * So this vector points left -> right, which is sideways.
     * Robot forward heading = baseline heading - 90 degrees.
     */

    double rel_n = static_cast<double>(msg->rel_pos_n);
    double rel_e = static_cast<double>(msg->rel_pos_e);

    if (rel_n == 0.0 && rel_e == 0.0) {
        RCLCPP_WARN(this->get_logger(), "RELPOSNED baseline is zero; cannot compute heading.");
        return;
    }

    double baseline_heading = std::atan2(rel_e, rel_n) * 180.0 / M_PI;
    baseline_heading = normalizeHeading(baseline_heading);

    double heading_angle = normalizeHeading(baseline_heading - 90.0);

    std::string heading_dir = "N/A";

    if ((heading_angle >= 337.5 || heading_angle < 22.5)) heading_dir = "N";
    else if (heading_angle < 67.5) heading_dir = "NE";
    else if (heading_angle < 112.5) heading_dir = "E";
    else if (heading_angle < 157.5) heading_dir = "SE";
    else if (heading_angle < 202.5) heading_dir = "S";
    else if (heading_angle < 247.5) heading_dir = "SW";
    else if (heading_angle < 292.5) heading_dir = "W";
    else if (heading_angle < 337.5) heading_dir = "NW";

    RCLCPP_INFO(
        this->get_logger(),
        "Heading Direction: %s, Heading Angle: %.2f, Baseline Heading: %.2f",
        heading_dir.c_str(),
        heading_angle,
        baseline_heading
    );
}