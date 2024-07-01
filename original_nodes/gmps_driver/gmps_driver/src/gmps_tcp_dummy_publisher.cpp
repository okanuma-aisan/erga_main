#include <rclcpp/rclcpp.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class GMPSDummyPublisher : public rclcpp::Node
{
private://ros timer
	rclcpp::TimerBase::SharedPtr timer_;

private://GMPS用変数
	std::string dummy_data = "F,5,0.501,-100.123,S";
	std::string gmps_ip_;//GMPSデバイスのIP
	int gmps_port_;//GMPSデバイスのport
	bool connect_f_ = false;//GMPSデバイスに接続するとtrue
	int sock_ = -1;//ソケットインスタンスID
	int client_sock_ = 1;//クライアントソケットインスタンスID
	int maxfd_ = 1;

private://ros callback
	void callbackTimer()
	{
		if(connect_f_ == false)//GMPSデバイスに接続していない場合は接続処理を行う
		{
			printf("Connetcing\n");
			bool connected = mag_pc_connect();

			if (connected == true)
			{
				//FD_ZERO(&readfds_);
				//FD_SET(sock_, &readfds_);
				maxfd_ = sock_;

				//cmd_send(sock,"START 0");
			} 	
			
			rclcpp::Rate rate(1);
			rate.sleep();
		}
		else
		{
			cmd_send();
		}
	}

private:
	//GMPSドライバに接続する
	bool mag_pc_connect()
	{
		bool ret_bo = false;
		int ret;
		struct sockaddr_in addr;
		connect_f_ = false;

		sock_ = socket(AF_INET, SOCK_STREAM, 0);
		addr.sin_family = AF_INET;
		addr.sin_port = htons(gmps_port_);//htons(9501);
		addr.sin_addr.s_addr = INADDR_ANY;//INADDR_ANY;//inet_addr(gmps_ip_.c_str());

		//ret = connect(sock_,(struct sockaddr *)&server,sizeof(server));
		std::cout << "binding" << std::endl;
		ret = bind( sock_, (struct sockaddr *)&addr, sizeof( addr ) );
		if (ret < 0)
		{
			std::cerr << "error bind , " << errno << std::endl;
			return false;
		} 

		// 受信待ち
		std::cout << "listen" << std::endl;
		ret = listen( sock_, 3);//SOMAXCONN );
		if(ret < 0)
		{
			std::cerr << "error listen , " << errno << std::endl;
			close(sock_);
			return false;
		}

		// クライアントからのコネクト要求待ち
		//struct sockaddr_in from_addr;
		//socklen_t len = sizeof( struct sockaddr_in );
		std::cout << "accepting" << std::endl;
		client_sock_ = accept( sock_, nullptr, nullptr);//(struct sockaddr *)&from_addr, &len );
		if(client_sock_ < 0)
		{
			std::cerr << "error accept , " << errno << std::endl;
			close(sock_);
			return false;
		}

		std::cout << "gmps-board-pc Connected" << std::endl;
		connect_f_ = true;
		return true;
	}

	void cmd_send()
	{
		ssize_t ret = write(client_sock_, dummy_data.c_str(), dummy_data.length());
		if(ret == -1)
		{
			std::cerr << "error write , " << errno << std::endl;
		}
	}

public:
	GMPSDummyPublisher(const rclcpp::NodeOptions & node_options)
		: rclcpp::Node("gmps_tcp_dummy_publisher", node_options)
	{
		this->declare_parameter<std::string>("gmps_ip", "192.168.1.20");
		gmps_ip_ = this->get_parameter("gmps_ip").as_string();
		this->declare_parameter<uint16_t>("gmps_port", 9501);
		gmps_port_ = static_cast<uint16_t>(this->get_parameter("gmps_port").as_int());

		timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Rate(100).period(),
			std::bind(&GMPSDummyPublisher::callbackTimer, this));
	}

	~GMPSDummyPublisher()
	{
		close(client_sock_);
		close(sock_);
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	auto node = std::make_shared<GMPSDummyPublisher>(node_options);
	rclcpp::spin(node);
	return 0;
}