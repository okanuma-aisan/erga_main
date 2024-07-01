#include <rclcpp/rclcpp.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <gmps_msgs/msg/gmps_error.hpp>

class GMPSUdpPub : public rclcpp::Node
{
private://UDP関連
	int sock_ = -1;
	struct sockaddr_in addr_;

private://ros subscriber
	rclcpp::Subscription<gmps_msgs::msg::GmpsError>::SharedPtr sub_gmps_error_;//GMPSデバイスのエラー情報
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gmps_pose_;//GMPSデバイスから得られた車両位置情報
	rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_;//Twist情報 これはGMPS以外のデバイスから取得する

private://gmpsデバイス情報
	gmps_msgs::msg::GmpsError gmps_error_;//GMPSデバイスのエラー情報
	geometry_msgs::msg::TwistWithCovarianceStamped twist_;//Twist情報 これはGMPS以外のデバイスから取得する

private://ros callback
	void callbackGmpsError(const gmps_msgs::msg::GmpsError::SharedPtr error)
	{
		gmps_error_ = *error;
	}

	void callbackGmpsPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
	{
		char buf[sizeof(double)*(7+6) + sizeof(uint8_t)];
		double* dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*0]);
		*dtmp = pose->pose.pose.position.x;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*1]);
		*dtmp = pose->pose.pose.position.y;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*2]);
		*dtmp = pose->pose.pose.position.z;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*3]);
		*dtmp = pose->pose.pose.orientation.x;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*4]);
		*dtmp = pose->pose.pose.orientation.y;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*5]);
		*dtmp = pose->pose.pose.orientation.z;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*6]);
		*dtmp = pose->pose.pose.orientation.w;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*7]);
		*dtmp = twist_.twist.twist.linear.x;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*8]);
		*dtmp = twist_.twist.twist.linear.y;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*9]);
		*dtmp = twist_.twist.twist.linear.z;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*10]);
		*dtmp = twist_.twist.twist.angular.x;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*11]);
		*dtmp = twist_.twist.twist.angular.y;
		dtmp = reinterpret_cast<double*>(&buf[sizeof(double)*12]);
		*dtmp = twist_.twist.twist.angular.z;
		buf[sizeof(double)*12+sizeof(uint8_t)] = gmps_error_.error_number;

		ssize_t result = sendto(sock_, reinterpret_cast<void*>(buf), sizeof(double)*(7+6) + sizeof(uint8_t),
			0, (struct sockaddr*)&addr_, sizeof(struct sockaddr));
		if(result == -1)
			RCLCPP_ERROR(this->get_logger(), "error : udp send");
		//else
		//	RCLCPP_INFO(this->get_logger(), "ok : udp send");
	}

	void callbackTwist(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist)
	{
		twist_ = *twist;
	}

public:
	GMPSUdpPub(const rclcpp::NodeOptions &node_options)
		: rclcpp::Node("gmps_udp_pub", node_options)
	{
		std::string ip = this->declare_parameter<std::string>("ip", "192.168.1.20");
		int port = this->declare_parameter<int>("port", 4005);

		sub_gmps_error_ = this->create_subscription<gmps_msgs::msg::GmpsError>("gmps_error", rclcpp::QoS(1),
			std::bind(&GMPSUdpPub::callbackGmpsError, this, std::placeholders::_1));
		sub_gmps_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_aichi_pose", rclcpp::QoS(1),
			std::bind(&GMPSUdpPub::callbackGmpsPose, this, std::placeholders::_1));
		sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("twist", rclcpp::QoS(1),
			std::bind(&GMPSUdpPub::callbackTwist, this, std::placeholders::_1));

		sock_ = socket(AF_INET, SOCK_DGRAM, 0);
		if(sock_ == -1)
		{
			RCLCPP_ERROR(this->get_logger(), "error : udp open");
			return;
		}

		addr_.sin_family = AF_INET;
		addr_.sin_addr.s_addr = inet_addr(ip.c_str());
		addr_.sin_port = htons(port);

		RCLCPP_INFO(this->get_logger(), "udp connect ok");
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	std::shared_ptr<GMPSUdpPub> node = std::make_shared<GMPSUdpPub>(options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}