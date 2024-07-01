#include <rclcpp/rclcpp.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>

class NovatelOem7UdpSub : public rclcpp::Node
{
private:
	int sock_;
	struct sockaddr_in addr_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_sentence_;

public:
	NovatelOem7UdpSub()
		: Node("novatel_oem7_upd_sub")
	{
		sock_ = socket(AF_INET, SOCK_DGRAM, 0);
		if(sock_ == -1)
		{
			std::cerr << "error : socket open" << std::endl;
			return;
		}

		addr_.sin_family = AF_INET;
		addr_.sin_addr.s_addr = inet_addr("0.0.0.0");
		addr_.sin_port = htons(4003);

		bind(sock_, (const struct sockaddr *)&addr_, sizeof(addr_));

		pub_sentence_ = this->create_publisher<std_msgs::msg::String>("sentence" ,rclcpp::SensorDataQoS());
	}

	void sub()
	{
		const size_t BUFSIZE = 300;
		char buf[BUFSIZE];
		memset(buf, 0, sizeof(buf));

		ssize_t readsize = recv(sock_, buf, sizeof(buf), 0);

		if(readsize > 0)
		{
			std_msgs::msg::String nmea_pub_string;
			nmea_pub_string.data.append(buf);
			pub_sentence_->publish(nmea_pub_string);
		}
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<NovatelOem7UdpSub> driver = std::make_shared<NovatelOem7UdpSub>();
	while(rclcpp::ok())
	{
		//rclcpp::spin_some(driver);
		driver->sub();
	}
	rclcpp::shutdown();
	return 0;
}