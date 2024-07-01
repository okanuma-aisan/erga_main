#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <iomanip>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "nav_msgs/msg/odometry.hpp"
#include "tier4_map_msgs/msg/map_projector_info.hpp"
#include "tier4_api_msgs/msg/euler.hpp"

#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>


using std::placeholders::_1;

std::string autoware_to_mgrs(std::string mgrs_grid, double x, double y) {
  std::stringstream ss_x;
  std::stringstream ss_y;
  std::stringstream ss_mgrs;

  // mgrs maximum precision is 11
  ss_x << std::fixed << std::setw(12) << std::setfill('0') << std::setprecision(6) << x;
  ss_y << std::fixed << std::setw(12) << std::setfill('0') << std::setprecision(6) << y;
  ss_mgrs << mgrs_grid << ss_x.str() << ss_y.str();
  std::string ss_mgrs_str = ss_mgrs.str();
  // remove decimal dots
  ss_mgrs_str.erase(std::remove(ss_mgrs_str.begin(), ss_mgrs_str.end(), '.'), ss_mgrs_str.end());
  return ss_mgrs_str;
}

class PoseConversionNode : public rclcpp::Node
{
  public:
    PoseConversionNode()
    : Node("mgrs_gcs_converter"), map_info_received_(false)
    {

        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliable().transient_local();
        map_projection_subscription_ = this->create_subscription<tier4_map_msgs::msg::MapProjectorInfo>(
        "~/input_map_info", qos_profile, std::bind(&PoseConversionNode::map_projection_callback_, this, _1));

        kinematic_state_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/input_odometry_mgrs", 10, std::bind(&PoseConversionNode::kinematic_state_callback_, this, _1));

        converted_topic_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "~/output_odometry_gcs", 10);

        rpy_topic_publisher_ = this->create_publisher<tier4_api_msgs::msg::Euler>(
        "~/output_odometry_rpy", 10);
    }

  private:
    rclcpp::Subscription<tier4_map_msgs::msg::MapProjectorInfo>::SharedPtr map_projection_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr converted_topic_publisher_;
    rclcpp::Publisher<tier4_api_msgs::msg::Euler>::SharedPtr rpy_topic_publisher_;

    bool map_info_received_;
    tier4_map_msgs::msg::MapProjectorInfo map_info_;
    
    void map_projection_callback_(const tier4_map_msgs::msg::MapProjectorInfo & msg)
    {
        RCLCPP_INFO(this->get_logger(), "Map info received.");

        if (msg.projector_type != tier4_map_msgs::msg::MapProjectorInfo::MGRS) {
            RCLCPP_ERROR(this->get_logger(), "Map projection type is not MGRS. Unable to start conversion.");
            return;
        }

        if (msg.mgrs_grid.length() != 5) {
            RCLCPP_ERROR(this->get_logger(), "MGRS grid declaration is of invalid length. Unable to start conversion.");
            return;
        }

        this->map_info_received_ = true;
        this->map_info_ = msg;
        RCLCPP_INFO(this->get_logger(), "Map info is valid, able to start conversion.");
    }

    void kinematic_state_callback_(const nav_msgs::msg::Odometry & msg)
    {
        if (!this->map_info_received_) {
            RCLCPP_ERROR(this->get_logger(), "Attempted conversion, but map projection info is missing or invalid.");
            return;
        }
        auto converted_msg = nav_msgs::msg::Odometry();

        // copy from original message
        converted_msg.header = msg.header;
        converted_msg.child_frame_id = msg.child_frame_id;
        converted_msg.pose = msg.pose;
        converted_msg.twist = msg.twist;

        // convert from x, y and map info to mgrs string for passing to GeographicLib
        std::string mgrs_string = autoware_to_mgrs(this->map_info_.mgrs_grid, msg.pose.pose.position.x, msg.pose.pose.position.y);
        
        // GeographicLib::MGRS -> GeographicLib::UTMUPS
        int zone;
        bool northp;
        double x, y;
        int prec;
        GeographicLib::MGRS::Reverse(mgrs_string, zone, northp, x, y, prec);

        double lat, lon;
        GeographicLib::UTMUPS::Reverse(zone, northp, x, y, lat, lon, false);

        converted_msg.pose.pose.position.x = lat;
        converted_msg.pose.pose.position.y = lon;
        converted_msg.pose.pose.position.z = msg.pose.pose.position.z;

        tf2::Quaternion q(
          msg.pose.pose.orientation.x,
          msg.pose.pose.orientation.y,
          msg.pose.pose.orientation.z,
          msg.pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        auto rpy_msg = tier4_api_msgs::msg::Euler();
        rpy_msg.roll = roll;
        rpy_msg.pitch = pitch;
        rpy_msg.yaw = yaw;

        rpy_topic_publisher_->publish(rpy_msg);
        converted_topic_publisher_->publish(converted_msg);
    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseConversionNode>());
  rclcpp::shutdown();
  return 0;
}