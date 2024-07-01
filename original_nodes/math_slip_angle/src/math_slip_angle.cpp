#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//ユークリッド距離を計算
double euclidDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
{
	double x = p1.x - p2.x;
	double y = p1.y - p2.y;
	double z = p1.z - p2.z;
	return std::sqrt(x*x + y*y + z*z);
}

class MathSlipAngle : public rclcpp::Node
{
private:
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_curr_pos_;
	rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_curr_twist_;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_slip_yaw_deg_;

	geometry_msgs::msg::PoseWithCovarianceStamped prev_curr_pos_;

	double curr_vel_;

	void callbackCurrPos(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pos)
	{
		std_msgs::msg::Float64 msg_formed_yaw;

		if(curr_vel_ > 0.5)
		{
			double progression_yaw = std::atan2(pos->pose.pose.position.y - prev_curr_pos_.pose.pose.position.y,
				pos->pose.pose.position.x - prev_curr_pos_.pose.pose.position.x);
			tf2::Quaternion progression_qua;
			progression_qua.setRPY(0.0, 0.0, progression_yaw);

			tf2::Quaternion pos_qua;
			tf2::fromMsg(pos->pose.pose.orientation, pos_qua);

			tf2::Quaternion formed_angle = pos_qua * progression_qua.inverse();
			double formed_roll, formed_pitch, formed_yaw;
			tf2::Matrix3x3(formed_angle).getRPY(formed_roll, formed_pitch, formed_yaw);

			msg_formed_yaw.data = formed_yaw*180.0/M_PI;
		}
		else
			msg_formed_yaw.data = 0;

		pub_slip_yaw_deg_->publish(msg_formed_yaw);

		prev_curr_pos_ = *pos;
	}

	void callbackCurrTwist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist)
	{
		curr_vel_ = twist->twist.twist.linear.x;
	}

public:
	MathSlipAngle(const rclcpp::NodeOptions &node_options)
		: rclcpp::Node("math_slip_angle", node_options)
		, curr_vel_(0)
	{
		sub_curr_pos_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("curr_pos", rclcpp::QoS(5),
			std::bind(&MathSlipAngle::callbackCurrPos, this, std::placeholders::_1));
		sub_curr_twist_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("curr_twist", rclcpp::QoS(5),
			std::bind(&MathSlipAngle::callbackCurrTwist, this, std::placeholders::_1));

		pub_slip_yaw_deg_ = create_publisher<std_msgs::msg::Float64>("slip_yaw_deg", rclcpp::QoS(1));
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<MathSlipAngle> node = std::make_shared<MathSlipAngle>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}