#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <joyboard_kvaser_connector/joyboard_kvaser_connector.hpp>
#include <wada_vmc_msgs/msg/can_buffer.hpp>

namespace wada_vmc
{
	//kvaserを介してジョイボードからmessageを受信するクラス
	class KvaserReceiver : public rclcpp::Node
	{
	private://メンバ変数
		std::unique_ptr<joyboard_kvaser::JoyboardKvaserConnector> connector_;//kvaserデバイスとのcan通信を行うクラスインスタンスのポインタ
		unsigned long wait_time_ms_;//message受信のtimeout時間
		bool show_error_;

	private://ros2 publisher
		rclcpp::Publisher<wada_vmc_msgs::msg::CanBuffer>::SharedPtr pub_can_receive_;//joyボード受信データのpulisher

	public://ros2コールバック
		//joyボードからデータを受信する関数
		void recv()
		{
			//static rclcpp::Time prev_rostime = this->now();

			if(connector_->isConnect() == false)
			{
				if (show_error_ == false) return;
				show_error_ = false;

				RCLCPP_ERROR(this->get_logger(), "kvaser_receiver error : not connect");
				return;
			}

			//message受信
			unsigned char receive_buffer[100];
			long message_id;
			unsigned int message_lenght;
			canStatus stat = connector_->receive(receive_buffer, wait_time_ms_, message_id, message_lenght);
			if(stat != canStatus::canOK)
			{
				RCLCPP_ERROR(this->get_logger(), "kvaser_receiver error : receive error : %d", stat);
				return ;
			}

			//受信message長を確認
			if(message_lenght != wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT)
			{
				RCLCPP_ERROR(this->get_logger(), "kvaser_receiver error : not message lenght 8");
				return;
			}

			//受信したmessageをトピックとしてpublish
			wada_vmc_msgs::msg::CanBuffer::UniquePtr can_receive = std::make_unique<wada_vmc_msgs::msg::CanBuffer>();
			can_receive->stamp = this->now();
			can_receive->message_id = message_id;
			for(size_t i=0; i<wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT; i++)
				can_receive->receive_data[i] = receive_buffer[i];
			//RCLCPP_INFO(this->get_logger(), "1,%lu", (unsigned long)&can_receive->receive_data[0]);
			pub_can_receive_->publish(std::move(can_receive));
		}

	public:
		KvaserReceiver(const rclcpp::NodeOptions &node_option)
			: rclcpp::Node("kvaser_receiver", node_option)
			, connector_(nullptr)
			, show_error_(true)
		{
			this->declare_parameter<int>("serial1", 0);//kvaserシリアル番号1
			this->declare_parameter<int>("serial2", 0);//kvaserシリアル番号2
			this->declare_parameter("wait_time_ms", 10);//タイムアウト時間(ミリ秒)
			unsigned int serial1 = static_cast<unsigned int>(this->get_parameter("serial1").as_int());
			unsigned int serial2 = static_cast<unsigned int>(this->get_parameter("serial2").as_int());
			wait_time_ms_ = static_cast<unsigned long>(this->get_parameter("wait_time_ms").as_int()); 
			RCLCPP_INFO(this->get_logger(), "serial1:%d,%d", serial1, serial2);
			RCLCPP_INFO(this->get_logger(), "wait_time:%ld", wait_time_ms_);

			connector_ = std::make_unique<joyboard_kvaser::JoyboardKvaserConnector>(serial1, serial2);

			pub_can_receive_ = this->create_publisher<wada_vmc_msgs::msg::CanBuffer>("can_receive", rclcpp::SensorDataQoS());
		}
	};
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<wada_vmc::KvaserReceiver> node = std::make_shared<wada_vmc::KvaserReceiver>(node_options);
	while(rclcpp::ok())
	{
		rclcpp::spin_some(node);
		node->recv();
	}
	rclcpp::shutdown();
	return 0;
}
