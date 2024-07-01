#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

class PoseTwistToBaselink : public rclcpp::Node
{
private:
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
	rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ndt_pose_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_kinematic_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

	std::string tf_name_;

	geometry_msgs::msg::TwistWithCovarianceStamped twist_;
	sensor_msgs::msg::Imu imu_;
	//geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr ndt_pose_;

	void callbackPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
	{
		// RCLCPP_INFO(this->get_logger(), "pose");

		geometry_msgs::msg::PoseStamped base_pose;
		base_pose.header.stamp = msg->header.stamp;
		base_pose.header.frame_id = msg->header.frame_id;
		base_pose.pose = msg->pose.pose;
		/*if(ndt_pose_ != nullptr)
		{
			base_pose.pose.orientation = ndt_pose_->pose.pose.orientation;
		}*/
		pub_pose_->publish(base_pose);

		geometry_msgs::msg::PoseWithCovarianceStamped base_pose_cov;
		base_pose_cov.header = base_pose.header;
		base_pose_cov.pose = msg->pose;
		pub_pose_cov_->publish(base_pose_cov);

		nav_msgs::msg::Odometry kinematic;
		kinematic.header = base_pose.header;
		kinematic.child_frame_id = tf_name_;
		kinematic.pose = base_pose_cov.pose;
		kinematic.twist = twist_.twist;
		pub_kinematic_->publish(kinematic);

		geometry_msgs::msg::TransformStamped transform;
		transform.header = base_pose.header;
		transform.child_frame_id = tf_name_;
		transform.transform.translation.x = msg->pose.pose.position.x;
		transform.transform.translation.y = msg->pose.pose.position.y;
		transform.transform.translation.z = msg->pose.pose.position.z;
		transform.transform.rotation.x = msg->pose.pose.orientation.x;
		transform.transform.rotation.y = msg->pose.pose.orientation.y;
		transform.transform.rotation.z = msg->pose.pose.orientation.z;
		transform.transform.rotation.w = msg->pose.pose.orientation.w;
		tf_br_->sendTransform(transform);
	}

	void callbackTwist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
	{
		//RCLCPP_INFO(this->get_logger(), "twist");
		twist_ = *msg;
		twist_.twist.twist.angular = imu_.angular_velocity;
	}

	/*void callbackNdtPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
	{
		ndt_pose_ = msg;
	}*/

	void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
	{
		imu_ = *msg;
	}

public:
	PoseTwistToBaselink(const rclcpp::NodeOptions &options)
		: rclcpp::Node("pose_twist_to_baselink", options)
		//, ndt_pose_(nullptr)
	{
		tf_name_ = this->declare_parameter<std::string>("tf_name", "gnss_base_link");

		sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"input_pose", rclcpp::QoS{1}, std::bind(&PoseTwistToBaselink::callbackPose, this, std::placeholders::_1));
		sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
			"input_twist", rclcpp::QoS{1}, std::bind(&PoseTwistToBaselink::callbackTwist, this, std::placeholders::_1));
		//sub_ndt_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		//	"/localization/pose_with_covariance", rclcpp::QoS{1}, std::bind(&PoseTwistToBaselink::callbackNdtPose, this, std::placeholders::_1));
		sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
			"input_imu", rclcpp::QoS{1}, std::bind(&PoseTwistToBaselink::callbackImu, this, std::placeholders::_1));

		pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
			"gnss_baselink_pose", rclcpp::QoS{10});
		pub_pose_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"gnss_baselink_pose_cov", rclcpp::QoS{10});
		pub_kinematic_ = this->create_publisher<nav_msgs::msg::Odometry>(
			"kinematic_state_gnss", rclcpp::QoS{10});

		tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    		std::shared_ptr<rclcpp::Node>(this, [](auto) {}));
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<PoseTwistToBaselink> gtb = std::make_shared<PoseTwistToBaselink>(node_options);
	rclcpp::spin(gtb);
	rclcpp::shutdown();
	return 0;
}
