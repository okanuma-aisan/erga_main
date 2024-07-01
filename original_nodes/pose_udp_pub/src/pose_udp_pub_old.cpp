#include <rclcpp/rclcpp.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>

class PoseUdpPub : public rclcpp::Node
{
private:
	int sock_;
	struct sockaddr_in addr_;

	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
	rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_;
	rclcpp::Subscription<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr sub_ndt_nearest_voxel_transformation_likelihood_;

	geometry_msgs::msg::PoseWithCovarianceStamped pose_;
	void callbackPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
	{
		pose_ = *pose;
	}

	void callbackTwist(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist)
	{
		//if(pose_ == nullptr || ndt_nearest_voxel_transformation_likelihood_ == nullptr)
		//	return;

		const size_t POSE_DOUBLE_SIZE = 7;
		const size_t TWIST_DOUBLE_SIZE = 6;
		const size_t LIKELIHOOD_DOUBLE_SIZE = 1;

		double buf[POSE_DOUBLE_SIZE + TWIST_DOUBLE_SIZE + LIKELIHOOD_DOUBLE_SIZE];
		buf[0] = pose_.pose.pose.position.x;
		buf[1] = pose_.pose.pose.position.y;
		buf[2] = pose_.pose.pose.position.z;
		buf[3] = pose_.pose.pose.orientation.x;
		buf[4] = pose_.pose.pose.orientation.y;
		buf[5] = pose_.pose.pose.orientation.z;
		buf[6] = pose_.pose.pose.orientation.w;
		buf[7] = twist->twist.twist.linear.x;
		buf[8] = twist->twist.twist.linear.y;
		buf[9] = twist->twist.twist.linear.z;
		buf[10] = twist->twist.twist.angular.x;
		buf[11] = twist->twist.twist.angular.y;
		buf[12] = twist->twist.twist.angular.z;
		buf[13] = static_cast<double>(ndt_nearest_voxel_transformation_likelihood_.data);

		ssize_t result = sendto(sock_, reinterpret_cast<void*>(buf), sizeof(double)*(POSE_DOUBLE_SIZE + TWIST_DOUBLE_SIZE + LIKELIHOOD_DOUBLE_SIZE),
			0, (struct sockaddr*)&addr_, sizeof(struct sockaddr));
		if(result == -1)
		{
			RCLCPP_ERROR(this->get_logger(), "error : udp send");
		}
		//RCLCPP_INFO(this->get_logger(), "pose send callback");
	}

	tier4_debug_msgs::msg::Float32Stamped ndt_nearest_voxel_transformation_likelihood_;
	void callbackNdtNearestVoxelTransformationLikelihood(const tier4_debug_msgs::msg::Float32Stamped::SharedPtr msg)
	{
		ndt_nearest_voxel_transformation_likelihood_ = *msg;
	}
public:
	PoseUdpPub()
		: rclcpp::Node("pose_udp_pub")
		, sock_(-1)
	{
		std::string ip = this->declare_parameter<std::string>("ip", "192.168.1.20");
		int port = this->declare_parameter<int>("port", 4001);

		sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"input_pose", rclcpp::QoS(1), std::bind(&PoseUdpPub::callbackPose, this, std::placeholders::_1));
		sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
			"input_twist", rclcpp::QoS(1), std::bind(&PoseUdpPub::callbackTwist, this, std::placeholders::_1));
		sub_ndt_nearest_voxel_transformation_likelihood_ = this->create_subscription<tier4_debug_msgs::msg::Float32Stamped>(
			"/localization/pose_estimator/nearest_voxel_transformation_likelihood",
			rclcpp::QoS(1), std::bind(&PoseUdpPub::callbackNdtNearestVoxelTransformationLikelihood, this, std::placeholders::_1));

		sock_ = socket(AF_INET, SOCK_DGRAM, 0);
		if(sock_ == -1)
		{
			RCLCPP_ERROR(this->get_logger(), "error : udp open");
			return;
		}

		addr_.sin_family = AF_INET;
		addr_.sin_addr.s_addr = inet_addr(ip.c_str());
		addr_.sin_port = htons(port);

		RCLCPP_INFO(this->get_logger(), "connect ok");
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<PoseUdpPub> pose_udp_pub = std::make_shared<PoseUdpPub>();
	rclcpp::spin(pose_udp_pub);
	rclcpp::shutdown();
	return 0;
}
