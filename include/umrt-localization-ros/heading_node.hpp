#ifndef HEADING_NODE_HPP
#define HEADING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_rel_pos_ned.hpp>
#include <string>

class HeadingNode : public rclcpp::Node {
public:
    HeadingNode();

private:
    rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>::SharedPtr relpos_sub;

    void relposCallback(
        const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg
    );

    double normalizeHeading(double heading_deg) const;
};

#endif // HEADING_NODE_HPP