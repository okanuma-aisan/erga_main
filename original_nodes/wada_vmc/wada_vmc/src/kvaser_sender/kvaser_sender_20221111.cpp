#include <rclcpp/rclcpp.hpp>
#include <joyboard_kvaser_connector/joyboard_kvaser_connector.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <wada_vmc_msgs/msg/can_buffer.hpp>
#include <wada_vmc_msgs/msg/can100_20221111.hpp>

namespace wada_vmc
{
	class KvaserSender20221111 : public rclcpp::Node
	{
	private://定数
		constexpr static int HZ = 100;//joyボードのsend周期 wada JOYボードの送受信周期は100HZ

   	private://メンバ変数
		std::unique_ptr<joyboard_kvaser::JoyboardKvaserConnector> connector_;//kvaserデバイスとのcan通信を行うクラスインスタンスのポインタ
		wada_vmc_msgs::msg::Can100_20221111::SharedPtr can100_;//joyに送信するID100 message
		bool send_error_;
		std::string select_can_device_;//使用するcanデバイス

	private://ros2 publisher
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_sender_hex_;//joy boardに送信するデータの16進表記

	private://ros2 subscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::Can100_20221111>::SharedPtr sub_can100_;//joyボード受信データ0x501のsuscriber

	private://ros2 timer
		rclcpp::TimerBase::SharedPtr send_timer_;//joyボードにデータを送信するタイマーコールバック

	private://ros2 callback
		void callbackCan100(const wada_vmc_msgs::msg::Can100_20221111::SharedPtr message100)
		{
			can100_ = message100;
		}

	private://ros2 timer


	private:
		inline uint8_t& GetByteInCANOrder(uint8_t* buffer, size_t n) {
	    		return buffer[7 - n];
		}
		inline void SetBit(uint8_t& byte, uint8_t bitPosition) {
    			byte |= (1 << bitPosition);
		}

		// 0バイト目の5ビット目をセット
		//SetBit(GetByteInCANOrder(buffer, 0), 5);

		//joyボードにデータを送信するタイマーコールバック
		void callbackSendTimer()
		{
			if(connector_->isConnect() == false)
			{
				if (send_error_ == false) return;
				send_error_ = false;
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
			//if(select_can_device_ == "pedal") buffer[0] |= can100_->pedal_control_mode << 4;
			//if(select_can_device_ == "steer") buffer[0] |= can100_->steer_control_mode;
			buffer[0] = 0xAA;

			//ペダル指令
			unsigned char *ptr_tmp = reinterpret_cast<unsigned char*>(&can100_->pedal_cmd);
			buffer[2] = ptr_tmp[1];  buffer[3] = ptr_tmp[0];

			//ハンドル角度指令
			int16_t handle_cmd_minus = can100_->handle_cmd;
			ptr_tmp = reinterpret_cast<unsigned char*>(&handle_cmd_minus);
			buffer[4] = ptr_tmp[1];  buffer[5] = ptr_tmp[0];

			//1byte目の7bitを1にすると、ペダル自動可になる。 この時、運転士がペダルセットボタンを押すと自動になる。
			if(can100_->pedal_auto_permission == true)  SetBit(GetByteInCANOrder(buffer, 1), 7);
			//1byte目の6bitを1にすると、ペダルフェールセイフになる（クラッチ断を保持）
			if(can100_->pedal_failsafe == true) 		SetBit(GetByteInCANOrder(buffer, 1), 6);
			//ブレーキのテイクオーバー保持非保持切り替え（falseで切断状態の保持）
			if(can100_->breke_takeover_latch == true) 	SetBit(GetByteInCANOrder(buffer, 1), 5);
			//0byte目の7bitを1にすると、ハンドル自動可になる。 この時、運転士がペダルセットボタンを押すと自動になる。
			if(can100_->handle_auto_permission == true) SetBit(GetByteInCANOrder(buffer, 0), 7);
			//0byte目の6bitを1にすると、ハンドルフェールセイフになる（クラッチ断を保持）
			if(can100_->handle_failsafe == true) 		SetBit(GetByteInCANOrder(buffer, 0), 6);


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
		KvaserSender20221111(const rclcpp::NodeOptions &node_option)
			: rclcpp::Node("wada_vmc_kvaser_sender", node_option)
			, connector_(nullptr)
			, can100_(nullptr)
			, send_error_(true)
		{
			this->declare_parameter<int>("serial1", 0);//kvaserシリアル番号1
			this->declare_parameter<int>("serial2", 0);//kvaserシリアル番号2
			this->declare_parameter<std::string>("select_can_device", "");
			unsigned int serial1 = static_cast<unsigned int>(this->get_parameter("serial1").as_int());
			unsigned int serial2 = static_cast<unsigned int>(this->get_parameter("serial2").as_int());
			select_can_device_ = get_parameter("select_can_device").as_string();

			RCLCPP_INFO(this->get_logger(), "serial,%u", serial1);
			connector_ = std::make_unique<joyboard_kvaser::JoyboardKvaserConnector>(serial1, serial2);

			pub_debug_sender_hex_ = this->create_publisher<std_msgs::msg::String>("sender_hex", rclcpp::QoS(1));

			std_msgs::msg::Bool firstlock;
			firstlock.data = false;

			unsigned char buffer[wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT] = {0,0,0,0,0,0,0,0};
			buffer[0] = 0x55;//ペダルもハンドルも実際は05ではなかったでしょうか？
			connector_->sende(buffer, 0x100, wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT);
			rclcpp::Rate rate(1.0/0.2);
			rate.sleep();
			buffer[0] = 0x00;
			connector_->sende(buffer, 0x100, wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT);
			rate.sleep();

			sub_can100_ = this->create_subscription<wada_vmc_msgs::msg::Can100_20221111>("can100", rclcpp::QoS(1),
				std::bind(&KvaserSender20221111::callbackCan100, this, std::placeholders::_1));

			//送信タイマーコールバック
			send_timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(HZ).period(),
				std::bind(&KvaserSender20221111::callbackSendTimer, this));
		}
	};
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<wada_vmc::KvaserSender20221111> node = std::make_shared<wada_vmc::KvaserSender20221111>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
