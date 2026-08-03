#include "umrt-localization-ros/heading_node.hpp"

#include <ublox_ubx_msgs/msg/carr_soln.hpp>

#include <cmath>
#include <string>

HeadingNode::HeadingNode() : Node("heading_node")
{
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.transient_local();

    this->declare_parameter<std::string>("relpos_topic", "/gps_starboard/ubx_nav_rel_pos_ned");
    const std::string relpos_topic = this->get_parameter("relpos_topic").as_string();
    relpos_sub = this->create_subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>(relpos_topic, qos, std::bind(&HeadingNode::relposCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Moving-base RTK heading node initialized.");
}

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

void HeadingNode::relposCallback(
    const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg
)
{
    /*
     * RELPOSNED gives the vector from the moving base antenna
     * to the rover antenna.
     *
     * Antenna mounting on this robot:
     *   left antenna  = base
     *   right antenna = rover
     *
     * So the baseline vector points left -> right (sideways).
     * Robot forward heading = baseline heading - 90 degrees.
     */

    // --- Gate on solution validity before trusting anything ---
    //
    // Field layout per ublox_ubx_msgs: validity flags are flat bools on the
    // message, and carr_soln is a CarrSoln sub-message with a .status field
    // plus named constants. Confirm with:
    //   ros2 interface show ublox_ubx_msgs/msg/UBXNavRelPosNED

    if (!msg->gnss_fix_ok || !msg->rel_pos_valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "RELPOSNED not valid yet (gnss_fix_ok=%d, rel_pos_valid=%d); skipping.", static_cast<int>(msg->gnss_fix_ok), static_cast<int>(msg->rel_pos_valid));
        return;
    }

    const uint8_t carr_soln = msg->carr_soln.status;
    if (carr_soln != ublox_ubx_msgs::msg::CarrSoln::CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES)
    {
        // 0 = no carrier solution, 1 = RTK float, 2 = RTK fixed.
        // Float headings can be off by several degrees; log but don't trust.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "RTK not fixed (carr_soln=%d); heading unreliable, skipping.", static_cast<int>(carr_soln));
        return;
    }

    // --- Build the baseline vector in meters, including HP components ---
    // rel_pos_n/e are in cm; rel_pos_hp_n/hp_e are in 0.1 mm.
    // Full HP value in cm = rel_pos_n + rel_pos_hp_n * 1e-2, then * 1e-2 for m.
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

    // --- Sanity check: baseline length vs. known antenna separation ---
    // Set this to your measured antenna-to-antenna distance in meters.
    // If the RTK length disagrees, the ambiguities fixed wrongly and the
    // heading is garbage (see u-blox app note UBX-19009093, section 2.4).
    constexpr double kExpectedBaselineM = 0.43;   // <-- EDIT to your robot
    constexpr double kBaselineToleranceM = 0.05;  // +/- 5 cm

    const double baseline_len = std::sqrt(rel_n * rel_n + rel_e * rel_e);
    if (std::fabs(baseline_len - kExpectedBaselineM) > kBaselineToleranceM) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Baseline length %.3f m differs from expected %.2f m; possible wrong ambiguity fix, skipping.", baseline_len, kExpectedBaselineM);
        return;
    }

    // --- Compute heading ---
    double baseline_heading =
        std::atan2(rel_e, rel_n) * 180.0 / M_PI;
    baseline_heading = normalizeHeading(baseline_heading);

    const double heading_angle = normalizeHeading(baseline_heading - 90.0);

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