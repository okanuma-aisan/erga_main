#include <rclcpp/rclcpp.hpp>
#include <joyboard_kvaser_connector/joyboard_kvaser_connector.hpp>
#include <std_msgs/msg/string.hpp>
#include <wada_vmc_msgs/msg/can_buffer.hpp>
#include <wada_vmc_msgs/msg/can100_20210103.hpp>

namespace wada_vmc
{
	class KvaserSender20210103 : public rclcpp::Node
	{
	private://定数
		constexpr static int HZ = 100;//joyボードのsend周期 wada JOYボードの送受信周期は100HZ

   	private://メンバ変数
		std::unique_ptr<joyboard_kvaser::JoyboardKvaserConnector> connector_;//kvaserデバイスとのcan通信を行うクラスインスタンスのポインタ
		wada_vmc_msgs::msg::Can100_20210103::SharedPtr can100_;//joyに送信するID100 message

	private://ros2 publisher
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_sender_hex_;

	private://ros2 subscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::Can100_20210103>::SharedPtr sub_can100_;//joyボード受信データ0x501のsuscriber

	private://ros2 timer
		rclcpp::TimerBase::SharedPtr send_timer_;//joyボードにデータを送信するタイマーコールバック

	private://ros2 callback
		void callbackCan100(const wada_vmc_msgs::msg::Can100_20210103::SharedPtr message100)
		{
			can100_ = message100;
		}

	private://ros2 timer
		//joyボードにデータを送信するタイマーコールバック
		void callbackSendTimer()
		{
			if(connector_->isConnect() == false)
			{
				RCLCPP_ERROR(this->get_logger(), "kvaser_sender error : not connect");
				return;
			}
			if(can100_ == nullptr)
			{
				RCLCPP_INFO(this->get_logger(), "not subscribe message ID 100");
				return;
			}

			//steerとpedalの自動・手動状態
			unsigned char buffer[wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT] = {0,0,0,0,0,0,0,0};
			buffer[0] = can100_->steer_control_mode << 4;
			buffer[0] += can100_->pedal_control_mode;

			//ハンドル角度指令
			unsigned char *ptr_tmp = reinterpret_cast<unsigned char*>(&can100_->steer_command);
			buffer[2] = ptr_tmp[1];  buffer[3] = ptr_tmp[0];

			//ペダル指令
			ptr_tmp = reinterpret_cast<unsigned char*>(&can100_->pedal_command);
			buffer[4] = ptr_tmp[1];  buffer[5] = ptr_tmp[0];

			//緊急ブレーキ
			if(can100_->emergency_brake == true) buffer[6] |= 0x80;

			//フェイルセーフON
			if(can100_->fail_safe_lighting == true) buffer[6] |= 0x40;

			//ペダルフェイルセーフ
			if(can100_->pedal_auto_permission == false) buffer[6] |= 0x20;

			//ステアフェイルセーフ
			if(can100_->handle_auto_permission == false) buffer[6] |= 0x10;

			//自動扉オープン
			if(can100_->automatic_door_open == true) buffer[6] |= 0x08;

			//自動停止未完了
			if(can100_->auto_stop_incomplete == true) buffer[6] |= 0x04;

			//ウィンカー左
			if(can100_->blinker_right == true) buffer[6] |= 0x02;

			//ウィンカー右
			if(can100_->blinker_left == true) buffer[6] |= 0x01;

			//ハザードランプ
			if(can100_->hazard_lamp) buffer[7] |= 0x80;

			//ウィンカーハザードキャンセル
			if(can100_->blinker_hazard_cancel == true) buffer[7] |= 0x40;

			//ステア自動禁止
			if(can100_->steer_automatic_ban == true) buffer[7] |= 0x20;

			//ペダル自動禁止
			if(can100_->pedal_automatic_ban == true) buffer[7] |= 0x10;

			//シフト自動禁止
			if(can100_->shift_automatic_ban == true) buffer[7] |= 0x08;

			//シフト状態
			if(can100_->shift_D == true) buffer[7] |= 0x01;
			else if(can100_->shift_N == true) buffer[7] |= 0x02;
			else if(can100_->shift_R == true) buffer[7] |= 0x04;

			connector_->sende(buffer, 0x100, wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT);

			publishReceiveString(buffer);
		}

	private:
		//joyボードに送信したデータを16進数表記でpublish
		//[in] receive_buffer:受信したmessage
		void publishReceiveString(const unsigned char* const receive_buffer)
		{
			std::stringstream ss;
			for(unsigned int i=0; i<wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT; i++)
			{
				ss << std::hex << +receive_buffer[i];
				if(i != wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT-1) ss << ',';
			}
			std_msgs::msg::String str;
			str.data = ss.str();
			pub_debug_sender_hex_->publish(str);
		}

	public:
		KvaserSender20210103(const rclcpp::NodeOptions &node_option)
			: rclcpp::Node("kvaser_sender", node_option)
			, connector_(nullptr)
			, can100_(nullptr)
		{
			unsigned int serial1 = this->declare_parameter("serial1", 0);//kvaserシリアル番号1
			unsigned int serial2 = this->declare_parameter("serial2", 0);//kvaserシリアル番号2
			RCLCPP_INFO(this->get_logger(), "serial,%u", serial1);
			connector_ = std::make_unique<joyboard_kvaser::JoyboardKvaserConnector>(serial1, serial2);

			unsigned char buffer[wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT] = {0,0,0,0,0,0,0,0};
			buffer[0] = 0x55;
			connector_->sende(buffer, 0x100, wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT);
			rclcpp::Rate rate(7.5);
			rate.sleep();
			buffer[0] = 0x00;
			connector_->sende(buffer, 0x100, wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT);
			rate.sleep();

			pub_debug_sender_hex_ = this->create_publisher<std_msgs::msg::String>("sender_hex", rclcpp::QoS(1));

			sub_can100_ = this->create_subscription<wada_vmc_msgs::msg::Can100_20210103>("can100", rclcpp::SensorDataQoS(),
				std::bind(&KvaserSender20210103::callbackCan100, this, std::placeholders::_1));

			//送信タイマーコールバック
			send_timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(HZ).period(),
				std::bind(&KvaserSender20210103::callbackSendTimer, this));
		}
	};
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<wada_vmc::KvaserSender20210103> node = std::make_shared<wada_vmc::KvaserSender20210103>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
