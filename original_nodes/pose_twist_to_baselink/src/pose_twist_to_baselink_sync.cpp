#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

class PoseTwistToBaselink : public rclcpp::Node
{
private:
	using PoseTwistSyncPolicy =
		message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseWithCovarianceStamped, geometry_msgs::msg::TwistWithCovarianceStamped>>;

	const size_t max_queue_size_ = 5;

	message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> sub_pose_;
	message_filters::Subscriber<geometry_msgs::msg::TwistWithCovarianceStamped> sub_twist_;
	std::shared_ptr<PoseTwistSyncPolicy> sync_pose_twist_;

	void callbackPoseTwist(
		const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose,
		const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist)
	/**{
		RCLCPP_INFO(this->get_logger(), "aaa");
	}**/
public:
	PoseTwistToBaselink() : rclcpp::Node("pose_tiwst_to_baselink")
	{
		sub_pose_.subscribe(this, "/sensing/gnss/pose_with_covariance",
			rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());
		sub_twist_.subscribe(this, "/vehicle/status/velocity_status",
			rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());

		sync_pose_twist_ = std::make_shared<PoseTwistSyncPolicy>(max_queue_size_);
		sync_pose_twist_->connectInput(sub_pose_, sub_twist_);
		sync_pose_twist_->registerCallback(std::bind(
			&PoseTwistToBaselink::callbackPoseTwist, this, std::placeholders::_1, std::placeholders::_2));
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<PoseTwistToBaselink> gtb = std::make_shared<PoseTwistToBaselink>();
	rclcpp::spin(gtb);
	rclcpp::shutdown();
	return 0;
}
