#include <rclcpp/rclcpp.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <wada_vmc_msgs/msg/can502_20221111.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
//#include <std_msgs/msg/float64.hpp>

class WadaVmcUdpPub : public rclcpp::Node
{
private://udp関連
   	int sock_;
	struct sockaddr_in addr_;

private://ros publisher
	rclcpp::Publisher<wada_vmc_msgs::msg::Can502_20221111>::SharedPtr pub_can502_;//joyボードのID0x502データ
	rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_velocity_report_;//autoware上の速度レポート
	rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steering_report_;//autoware上のステアレポート
	//rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_can502_;//joyボードのID0x502データ

public:
	WadaVmcUdpPub(const rclcpp::NodeOptions &options)
		: rclcpp::Node("wada_vmc_udp_sub", options)
	{
		std::string ip = this->declare_parameter<std::string>("udp_ip", "0.0.0.0");
		int port = this->declare_parameter<int>("udp_port", 4006);

		pub_can502_ = this->create_publisher<wada_vmc_msgs::msg::Can502_20221111>("/vehicle/steer/can502", rclcpp::QoS(1));
		pub_velocity_report_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("velocity_status", rclcpp::QoS{1});
		pub_steering_report_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("steering_status", rclcpp::QoS{1});
		//pub_can502_ = this->create_publisher<std_msgs::msg::Float64>("/sensing/wada_vmc/steer/can502", rclcpp::SensorDataQoS());

		sock_ = socket(AF_INET, SOCK_DGRAM, 0);
		if(sock_ == -1)
		{
			std::cerr << "error : socket open" << std::endl;
			return;
		}

		addr_.sin_family = AF_INET;
		addr_.sin_addr.s_addr = inet_addr(ip.c_str());
		addr_.sin_port = htons(port);

		bind(sock_, (const struct sockaddr *)&addr_, sizeof(addr_)); 
		RCLCPP_INFO(this->get_logger(), "connect ok");
	}

	void udpSub()
	{
		char buf[sizeof(double)*10];

		ssize_t readsize = recv(sock_, buf, sizeof(double)*2, 0);
		if(readsize == -1 || readsize != sizeof(double)*2)
		{
			std::cerr << "error : recv" << std::endl;
			return;
		}

		wada_vmc_msgs::msg::Can502_20221111 can502;
		can502.stamp = this->now();
		can502.tire_angle_rad = *(reinterpret_cast<double*>(&buf[sizeof(double)*0]));

		rclcpp::Time nowtime = this->now();
		autoware_auto_vehicle_msgs::msg::VelocityReport msg_velocity_report;
		msg_velocity_report.header.frame_id = "base_link";
		msg_velocity_report.header.stamp = nowtime;
		msg_velocity_report.longitudinal_velocity = *(reinterpret_cast<double*>(&buf[sizeof(double)*1]));
		if(msg_velocity_report.longitudinal_velocity < 0.1 / 3.6) msg_velocity_report.longitudinal_velocity = 0;
		msg_velocity_report.lateral_velocity = 0;
		msg_velocity_report.heading_rate = 0;
		pub_velocity_report_->publish(msg_velocity_report);
		//std_msgs::msg::Float64 can502;
		//can502.data = *(reinterpret_cast<double*>(&buf[sizeof(double)*0]));

		autoware_auto_vehicle_msgs::msg::SteeringReport msg_steering_report;
		msg_steering_report.stamp = nowtime;
		msg_steering_report.steering_tire_angle = can502.tire_angle_rad ;
		pub_steering_report_->publish(msg_steering_report);

		pub_can502_->publish(can502);
		RCLCPP_INFO(this->get_logger(), "publish ok , %s", pub_can502_->get_topic_name());
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	std::shared_ptr<WadaVmcUdpPub> node = std::make_shared<WadaVmcUdpPub>(options);
	while(rclcpp::ok())
	{
		//rclcpp::spin_some(node);
		node->udpSub();
	}
	rclcpp::shutdown();
	return 0;
}