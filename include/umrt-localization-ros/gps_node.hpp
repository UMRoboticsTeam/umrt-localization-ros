#ifndef GPS_NODE_HPP
#define GPS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_rel_pos_ned.hpp>

class GpsNode : public rclcpp::Node {
public:
    GpsNode();

private:
    rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>::SharedPtr relpos_sub;

    void relposCallback(
        const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg
    );

    double normalizeHeading(double heading_deg) const;
};

#endif // GPS_NODE_HPP