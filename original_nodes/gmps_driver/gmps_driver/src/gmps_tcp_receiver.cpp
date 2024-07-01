#include <rclcpp/rclcpp.hpp>
#include <gmps_msgs/msg/gmps_detect.hpp>
#include <gmps_msgs/msg/gmps_error.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

//GMPSパケットのメッセージタイプ
#define MSG_FRONT 1
#define MSG_REAR 2 //rearは不要かも
#define MSG_GMPSSTA 3
#define MSG_RFID 4
#define MSG_ERR 5
#define MSG_ACK_START 6
#define MSG_ACK_STOP 7
#define MSG_UNKNOWN -1

// パケットの分割
void msg_split(char* msg,std::vector<std::string>& items, int& items_cnt)
{

	std::stringstream ss;
	ss << msg;
	std::string item;

	while (std::getline(ss, item, ',')) {
		items.push_back(item);
	}

	items_cnt = items.size();

}


class GMPSReceiver : public rclcpp::Node
{
private://ros publisher
	rclcpp::Publisher<gmps_msgs::msg::GmpsDetect>::SharedPtr pub_gmps_detect_;//GMPSデバイスから取得した情報のpublisher
	rclcpp::Publisher<gmps_msgs::msg::GmpsError>::SharedPtr pub_gmps_error_;//GMPSデバイスからのエラー通知

private://ros timer
	rclcpp::TimerBase::SharedPtr gmpssta_timer_;//GMPSSTA受信時間監視用タイマー

private://GMPS用変数
	std::string gmps_ip_;//GMPSデバイスのIP
	int gmps_port_;//GMPSデバイスのport
	double gmpssta_timeout_;//GMPSSTAを受信エラーにするタイムアウト時間
	bool connect_f_ = false;//GMPSデバイスに接続するとtrue
	int sock_ = -1;//ソケットインスタンスID
	int maxfd_ = 1;
	fd_set fds_, readfds_;
	std::vector<std::string> items_;//受信したGMPSデータをカンマで分割した文字列群
	int item_type_;//現在受信したGMPSデータの種類
	bool measuring_f_ = false;
	rclcpp::Time gmpssta_receive_time_ = rclcpp::Time(1);//GMPSSTAを受信した時間
	bool error_pub_flag_ = false;//エラー情報をpublishしたらtrue

private://ros callback
	//GMPSSTA受信時間監視用タイマー
	void GMPSSTA_callback()
	{
		gmps_msgs::msg::GmpsError error;
		error.header.frame_id = "gmps";
		error.header.stamp = this->now();

		rclcpp::Duration rostimediff = this->now() - gmpssta_receive_time_;
		if(rostimediff.seconds() < gmpssta_timeout_)
		{
			if(error_pub_flag_ == false) error.error_number = gmps_msgs::msg::GmpsError::OK;
		}
		else
		{
			error.error_number = gmps_msgs::msg::GmpsError::ERR_GMPSSTA_TIMEOUT;
			error_pub_flag_ = true;
		}
		pub_gmps_error_->publish(error);
	}

private:
	//GMPSドライバに接続する
	bool mag_pc_connect()
	{
		int ret;
		struct sockaddr_in server;

		sock_ = socket(AF_INET, SOCK_STREAM, 0);
		if(ret < 0)
		{
			std::cerr << "error socket , " << errno << std::endl;
			return false;
		}

		server.sin_family = AF_INET;
		server.sin_port = htons(gmps_port_);//htons(9501);
		server.sin_addr.s_addr = inet_addr(gmps_ip_.c_str());
		//std::cout << gmps_ip_ << std::endl;
		RCLCPP_INFO(get_logger(), "gmps ip, ", gmps_ip_.c_str());
		ret = connect(sock_,(struct sockaddr *)&server,sizeof(server));
		if (ret < 0)
		{
			std::cerr <<"error connecting , " << errno << std::endl;
			close(sock_);
			return false;
		} 

		std::cerr <<"gmps-board-pc Connect" << std::endl;
		connect_f_ = true;
		return true;
	}

	//パケットの受信＆解析
	int cmd_recv()
	{
		char buf[100];
		int  r_len;

	//	std::vector<std::string> items;
		int items_cnt;

		// ボードPCからメッセージを受信
		memset(buf, 0, sizeof(buf));
		r_len = recv(sock_, buf, sizeof(buf), 0);
		if ( r_len < 0 )
		{
			printf("gmps-board-pc  Disconneted \n");
			connect_f_ = false ;
			return 0;//goto skip;
		}

		// パケットを分割 
		msg_split(buf,items_,items_cnt);

	#if 0

		std::cout <<  "data:";
		for (int i=0;i<items_cnt;i++)
		{
			std::cout << items_[i] << "\t";
		}
		std::cout <<  "\n";
		std::cout << "items_cnmt:" << items_cnt << std::endl;

	#endif

		// ==== パケット解析 =============================== 
		if (items_[0].compare("F") == 0)
		{
			//std::cout << buf << std::endl;
			RCLCPP_INFO(get_logger(), "buf, ", buf);
			if (items_cnt == 5)
			{
				//std::cout << items_[0] << "," << items_[1] << "," << items_[2] << "," << items_[3] << "," <<  items_[4] << std::endl;
				RCLCPP_INFO(get_logger(), "items,0,%s,1,%s,2,%s,3,%s,4,%s", items_[0].c_str(), items_[1].c_str(), items_[2].c_str(), items_[3].c_str(), items_[4].c_str());
				item_type_ = MSG_FRONT;
			}

		}
		else if (items_[0].compare("R") == 0)
		{

			if (items_cnt == 5)
			{
				std::cout << items_[0] << "\t" << items_[1] << "\t" << items_[2] << "\t" << items_[3] << "\t" <<  items_[4] << "\n";
				item_type_ = MSG_REAR;
			}

		}
		else if (items_[0].compare("GMPSSTA") == 0)
		{
			std::cout << "GMPSSTA" << std::endl;
			gmpssta_receive_time_ = this->now();
			item_type_ = MSG_GMPSSTA;
		}
		else if (items_[0].compare("RFID") == 0)
		{
			printf("RFID %s\n",items_[1].c_str());
			item_type_ = MSG_RFID;
		} 
		else if (items_[0].compare("ERR") == 0)
		{
			printf("ERR %s\n", items_[1].c_str());
			item_type_ = MSG_ERR;
		}
		else if (items_[0].compare("START") == 0)
		{
			// 成功 ? 
			if (items_[1].compare("OK") == 0)
			{
				std::cout << "START command OK" << std::endl;
				measuring_f_ = true;
			} else 
			{	// 失敗?
				std::cout << "ERR: START command " << items_[1].c_str() << std::endl;

				measuring_f_ = false;
			}
		} 
		else if (items_[0].compare("STOP") == 0)
		{
			// 成功 ? 
			if (items_[1].compare("OK") == 0)
			{
				printf("STOP command OK\n");
			} else 
			{	// 失敗?
				printf("ERR: STOP command %s\n",items_[1].c_str());
			}

			measuring_f_ = false;
		} 
		else 
		{
			printf("ERR: Unknown command %s\n",items_[0].c_str());
			item_type_ = MSG_UNKNOWN;
		}

	//skip:
		return 0;
	}
public:
	GMPSReceiver(const rclcpp::NodeOptions &node_option)
		: rclcpp::Node("gmps_tcp_receiver", node_option)
	{
		this->declare_parameter<std::string>("gmps_ip", "192.168.1.20");
		gmps_ip_ = this->get_parameter("gmps_ip").as_string();
		this->declare_parameter<uint16_t>("gmps_port", 9501);
		gmps_port_ = static_cast<uint16_t>(this->get_parameter("gmps_port").as_int());
		this->declare_parameter<double>("gmpssta_timeout", 0.5);
		gmpssta_timeout_ = static_cast<double>(this->get_parameter("gmpssta_timeout").as_double());

		pub_gmps_detect_ = this->create_publisher<gmps_msgs::msg::GmpsDetect>("gmps_detect", rclcpp::SensorDataQoS());
		pub_gmps_error_ = this->create_publisher<gmps_msgs::msg::GmpsError>("gmps_error",
			rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal));

		gmpssta_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Rate(30).period(), std::bind(&GMPSReceiver::GMPSSTA_callback, this));

		gmps_msgs::msg::GmpsError error;
		error.header.frame_id = "gmps";
		error.header.stamp = this->now();
		error.error_number = gmps_msgs::msg::GmpsError::ERR_INIT;
		pub_gmps_error_->publish(error);
	}

	void gmpsReceive()
	{
		if(connect_f_ == false)//GMPSデバイスに接続していない場合は接続処理を行う
		{
			printf("Connetcing\n");
			bool connected = mag_pc_connect();

			if (connected == true)
			{
				FD_ZERO(&readfds_);
				FD_SET(sock_, &readfds_);
				maxfd_ = sock_;

				//cmd_send(sock,"START 0");
			} 	
			
			rclcpp::Rate rate(1);
			rate.sleep();
		}
		else
		{
			items_.clear();

			memcpy(&fds_, &readfds_, sizeof(fd_set));
			select(maxfd_+1, &fds_, NULL, NULL, NULL);
			if (FD_ISSET(sock_, &fds_)) 
			{
				cmd_recv();
				//std::cout << "receive end" << std::endl;

                //Fの場合にmsgに格納してpublishする
                if (item_type_ == MSG_FRONT )
                {
					//std::cout << "receved" << std::endl;
					RCLCPP_INFO(get_logger(), "receved");

					gmps_msgs::msg::GmpsDetect msg_detect;

                    //items[0]はF/R
                    msg_detect.counter = std::stoi(items_[1]);
					//items[2]はdelaytime?
                    msg_detect.side_diff = std::stof(items_[3])/1000.0; //生値は[mm]なので[m]に変換する
                    if (items_[4].compare("N\n") == 0) //LF(0x10)が含まれている
                    {
                        msg_detect.pole = 1;
                    }
                    else if (items_[4].compare("S\n") == 0)
                    {
                        msg_detect.pole = 2;
                    }
                    msg_detect.mm_kind = 1; //マーカー種別の情報は1次試作、1.5次試作では使わない
                    msg_detect.delay_dist = 0.2; //1次試作、1.5次試作の検出遅延は20cm固定 //items[4]は使わない

					msg_detect.header.stamp = this->now();
                    pub_gmps_detect_->publish(msg_detect);
                }
				else if(item_type_ == MSG_ERR)
				{
					gmps_msgs::msg::GmpsError error;
					error.header.frame_id = "gmps";
					error.header.stamp = this->now();
					error.error_number = static_cast<uint8_t>(std::stoi(items_[1]));
					pub_gmps_error_->publish(error);
				}
			}
		}
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions option;
	std::shared_ptr<GMPSReceiver> node = std::make_shared<GMPSReceiver>(option);
	while (rclcpp::ok())
	{
		node->gmpsReceive();
	}
	rclcpp::shutdown();
	return 0;
}
