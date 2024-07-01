#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

//joyボード関連include
#include <wada_vmc_msgs/msg/can_buffer.hpp>
#include <wada_vmc_msgs/msg/can501_20221111.hpp>
#include <wada_vmc_msgs/msg/can502_20221111.hpp>

//autoware関連include
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>

namespace wada_vmc
{
	class ReceiveToTopic20221111 : public rclcpp::Node
	{
	//private://定数
	private://ros2 subscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::CanBuffer>::SharedPtr sub_can_receive_;//canメッセージのsubscriber

	private://ros2 publisher(joyボード関連)
		rclcpp::Publisher<wada_vmc_msgs::msg::Can501_20221111>::SharedPtr pub501_;//joyボード受信データ0x501のpulisher
		rclcpp::Publisher<wada_vmc_msgs::msg::Can502_20221111>::SharedPtr pub502_;//joyボード受信データ0x502のpulisher
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_receive_100_16_;//生のjoyボード受信データ(0x100)を16進数表記で配信するpublisher
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_receive_501_16_;//生のjoyボード受信データ(0x501)を16進数表記で配信するpublisher
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_receive_502_16_;//生のjoyボード受信データ(0x502)を16進数表記で配信するpublisher

	private://ros2 publisher(autoware関連)
		rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steering_status_rad_;//現在のホイール角度のpublisher 単位はrad autowareのcontrol系の処理に渡す

	private://車両固有情報
		//double wheelrad_to_steering_can_value_left_;//ホイール角度をjoyボードに送信するステア値に変換するための係数(左)
		//double wheelrad_to_steering_can_value_right_;//ホイール角度をjoyボードに送信するステア値に変換するための係数(右)
		double steering_can_value_left_to_tire_rad_slope_;
		double steering_can_value_right_to_tire_rad_slope_;
		double steering_can_value_left_to_tire_rad_intercept_;
		double steering_can_value_right_to_tire_rad_intercept_;
		int16_t steer_input_center_;//joyボードのステア中央値

	private://その他
		std::string select_can_device_;//"pedal"ならペダル情報のみをpublish "steer"ならステア情報のみをpublish

	private://ros2 callback
		//can_receiveトピックのsubscriber
		//message_id毎に各種トピックをpublishする
		//[in] can_receive:subscribeしたcan情報
		void callbackCanBuffer(const wada_vmc_msgs::msg::CanBuffer::SharedPtr can_receive)
		{
			//IDを確認して処理を決める
			switch(can_receive->message_id)
			{
			case 0x100:
				publishReceiveString(can_receive->message_id, can_receive->receive_data, pub_debug_receive_100_16_);
				break;	
			case 0x501:
				if(select_can_device_ == "pedal")
				{
					publish501(can_receive->receive_data);
					publishReceiveString(can_receive->message_id, can_receive->receive_data, pub_debug_receive_501_16_);
				}
				break;
			case 0x502:
				if(select_can_device_ == "steer")
				{
					publish502(can_receive->receive_data);
					publishReceiveString(can_receive->message_id, can_receive->receive_data, pub_debug_receive_502_16_);
				}
				break;
			}
		}

	private://受信メッセージをros2 topic化する関数群
		//message ID 501をpublish
		//[in] receive_buffer:受信したmessage
		void publish501(const std::array<uint8_t, 8> &receive_buffer)
		{
			wada_vmc_msgs::msg::Can501_20221111 can501;
			can501.stamp = this->now();

			can501.drive_mode = receive_buffer[0] >> 4;//ホストＰＣからのモード指令のアンサーバック（Ａ：自動運転モード、０：手動モード、５：通信未確立）

			uint8_t* tmp_ptr = reinterpret_cast<uint8_t*>(&can501.speed_kmh20);//車両速度12bitデータ：車両速度(km/h)*20のデータを送信
			tmp_ptr[1] = receive_buffer[0] & 0x0F;  tmp_ptr[0] = receive_buffer[1];

			can501.speed_kmh = can501.speed_kmh20 / 20;

			tmp_ptr = reinterpret_cast<uint8_t*>(&can501.pedal_pot_sub_v);//ホストPCからのペダル指令値の返信
			tmp_ptr[1] = receive_buffer[2] & 0x0F;  tmp_ptr[0] = receive_buffer[3];

			tmp_ptr = reinterpret_cast<uint8_t*>(&can501.pedal_pot_main_v);//ペダルPotの電圧測定デジタルデータ
			tmp_ptr[1] = receive_buffer[4] & 0x0F;  tmp_ptr[0] = receive_buffer[5];

			can501.pedal_pot_average = static_cast<int16_t>(((int)can501.pedal_pot_main_v + (int)can501.pedal_pot_sub_v + (double)0.5) / 2.0);//メインとサブの平均
			can501.pedal_pot_max = std::max(can501.pedal_pot_main_v, can501.pedal_pot_sub_v);
			/*if(can501.pedal_pot_average - 1018 > 0)
				can501.pedal_pot_max = (can501.pedal_pot_average-1018)*5.66+1018;
			else
				can501.pedal_pot_max = (can501.pedal_pot_average-1018)*(5.66*198.0/126.0)+1018;*/

			//true:自動運転モード false:マニュアルモード
			can501.auto_mode = (receive_buffer[6] & 0x80) ? true : false;

			//パルスモード  １：Ｐ１，２：Ｐ２，４：Ｐ４，８：Ｐ８，０：Ｐ１６
			can501.pulse_mode = (receive_buffer[6] & 0x78) >> 3;

			//空気圧異常
			can501.abnormal_air_pressure = (receive_buffer[6] & 0x04) ? true : false;

			//書き込み許可線ON
			can501.write_permission_on = (receive_buffer[6] & 0x02) ? true : false;

			//温度異常
			can501.temperature_anomaly = (receive_buffer[6] & 0x01) ? true : false;

			//過電流
			can501.over_current = (receive_buffer[7] & 0x80) ? true : false;

			//クラッチスイッチ状態
			can501.clutch = (receive_buffer[7] & 0x40) ? false : true;

			//ブレーキロックセット(ハンドルでは未使用、常に０)
			can501.brake_lock_set = (receive_buffer[7] & 0x20) ? true : false;

			//モーター線断線
			can501.motor_disconnection = (receive_buffer[7] & 0x10) ? true : false;

			//電源電圧低
			can501.power_supply_voltage_drop = (receive_buffer[7] & 0x08) ? true : false;

			//クラッチ線断線
			can501.clutch_disconnection = (receive_buffer[7] & 0x04) ? true : false;

			//メカポテンション断線
			can501.mecha_potentiometer_disconnection = (receive_buffer[7] & 0x02) ? true : false;

			//ジョイスティック断線
			can501.joy_disconnection = (receive_buffer[7] & 0x01) ? true : false;

			pub501_->publish(can501);
		}

		//message ID 502をpublish
		//[in] receive_buffer:受信したmessage
		void publish502(const std::array<uint8_t, 8> &receive_buffer)
		{
			rclcpp::Time nowtime = this->now();
			wada_vmc_msgs::msg::Can502_20221111 can502;
			can502.stamp = nowtime;

			can502.handle_mode = receive_buffer[0] >> 4;//ホストＰＣからのモード指令のアンサーバック（Ａ：自動運転モード、０：手動モード、５：通信未確立）

			uint8_t* tmp_ptr = reinterpret_cast<uint8_t*>(&can502.engin_rmp);//エンジンRPM
			tmp_ptr[1] = receive_buffer[0] & 0x0F;  tmp_ptr[0] = receive_buffer[1];

			can502.handle_do = receive_buffer[2] >> 4;//ハンドルDI

			tmp_ptr = reinterpret_cast<uint8_t*>(&can502.handle_pot_sub_v);//ホストPCからのステア指令値の返信
			tmp_ptr[1] = receive_buffer[2] & 0x0F;  tmp_ptr[0] = receive_buffer[3];
			//can502.handle_pot_sub_v += waypoint_param_.steer_actual_plus + waypoint_param_.steer_actual_plus_sub;//ハンドル補正

			can502.handle_di = receive_buffer[4] >> 4;//ハンドルDI

			tmp_ptr = reinterpret_cast<uint8_t*>(&can502.handle_pot_main_v);//ステアPotの電圧測定デジタルデータ
			tmp_ptr[1] = receive_buffer[4] & 0x0F;  tmp_ptr[0] = receive_buffer[5];
			//can502.handle_pot_main_v += waypoint_param_.steer_actual_plus + waypoint_param_.steer_actual_plus_sub;//ハンドル補正
#if 0		// 和田ボードの設定が左右で違う場合に使用
			if(can502.handle_pot_main_v >= 1024)
			{
				can502.handle_pot_main_v = static_cast<int16_t>((static_cast<double>(can502.handle_pot_main_v - 1024.0) * 920.0 / 802.0) + 1024.0);
				can502.handle_pot_sub_v = static_cast<int16_t>((static_cast<double>(can502.handle_pot_sub_v - 1024.0) * 920.0 / 802.0) + 1024.0);

			}
			else
			{
				can502.handle_pot_main_v = static_cast<int16_t>((static_cast<double>(can502.handle_pot_main_v - 1024.0) * 920.0 / 762.0) + 1024.0);
				can502.handle_pot_sub_v = static_cast<int16_t>((static_cast<double>(can502.handle_pot_sub_v - 1024.0) * 920.0 / 762.0) + 1024.0);
			}
#else
//			can502.handle_pot_main_v = static_cast<int16_t>((static_cast<double>(can502.handle_pot_main_v - 1024.0) * 920.0 / 762.0) + 1024.0);
//			can502.handle_pot_sub_v = static_cast<int16_t>((static_cast<double>(can502.handle_pot_sub_v - 1024.0) * 920.0 / 762.0) + 1024.0);
#endif
			can502.handle_pot_average = static_cast<int16_t>(((int)can502.handle_pot_main_v + (int)can502.handle_pot_sub_v + (double)0.5) / 2.0);//メインとサブの平均
			can502.handle_pot_max = std::max(can502.handle_pot_main_v,can502.handle_pot_sub_v);//メインとサブの最大

#if 0
			if(can502.handle_pot_main_v > steer_input_center_)//タイヤ角度の計算
			{
				can502.tire_angle_rad = (can502.handle_pot_main_v - steer_input_center_) * 
					steering_can_value_left_to_tire_rad_slope_ + steering_can_value_left_to_tire_rad_intercept_;
//				can502.tire_angle_rad = (can502.handle_pot_average - steer_input_center_) * 
//					steering_can_value_left_to_tire_rad_slope_ + steering_can_value_left_to_tire_rad_intercept_;
				RCLCPP_INFO(this->get_logger(), "left_angle,%lf", can502.tire_angle_rad);
			}
			else
			{
				can502.tire_angle_rad = (can502.handle_pot_main_v - steer_input_center_) * 
					steering_can_value_right_to_tire_rad_slope_ + steering_can_value_right_to_tire_rad_intercept_;
				RCLCPP_INFO(this->get_logger(), "right_angle,%lf", can502.tire_angle_rad);
			}
#else
			double handle_main = can502.handle_pot_main_v - steer_input_center_;
			can502.tire_angle_rad = std::pow(handle_main,5)*(-6.7e-17)+std::pow(handle_main,4)*(-8.2e-14)+std::pow(handle_main,3)*(1.26942e-11)+std::pow(handle_main,2)*6.74362e-8+handle_main*0.000898572-0.00463;
			RCLCPP_INFO(this->get_logger(), "right_angle,%lf", can502.tire_angle_rad);
#endif
			//true:自動運転モード false:マニュアルモード
			can502.auto_mode = (receive_buffer[6] & 0x40) ? true : false;

			//パルスモード  １：Ｐ１，２：Ｐ２，４：Ｐ４，８：Ｐ８，０：Ｐ１６
			can502.pulse_mode = (receive_buffer[6] & 0x3c) >> 2;

			//書き込み許可線ON
			can502.write_permission_on = (receive_buffer[6] & 0x02) ? true : false;

			//温度異常
			can502.temperature_anomaly = (receive_buffer[6] & 0x01) ? true : false;

			//過電流
			can502.over_current = (receive_buffer[7] & 0x80) ? true : false;

			//クラッチスイッチ状態
			can502.clutch = (receive_buffer[7] & 0x40) ? false : true;

			//ブレーキロックセット(ハンドルでは未使用、常に０)
			can502.brake_lock_set = (receive_buffer[7] & 0x20) ? true : false;

			//モーター線断線
			can502.motor_disconnection = (receive_buffer[7] & 0x10) ? true : false;

			//電源電圧低
			can502.power_supply_voltage_drop = (receive_buffer[7] & 0x08) ? true : false;

			//クラッチ線断線
			can502.clutch_disconnection = (receive_buffer[7] & 0x04) ? true : false;

			//メカポテンション断線
			can502.mecha_potentiometer_disconnection = (receive_buffer[7] & 0x02) ? true : false;

			//ジョイスティック断線
			can502.joy_disconnection = (receive_buffer[7] & 0x01) ? true : false;

			pub502_->publish(can502);

			//MPCに渡すステア情報をpublish
			autoware_auto_vehicle_msgs::msg::SteeringReport steering_report;
			steering_report.stamp = nowtime;
			steering_report.steering_tire_angle = can502.tire_angle_rad;
			pub_steering_status_rad_->publish(steering_report);
		}

	private://その他
		//joyボードから受信したデータを16進数表記でpublish
		//[in] message_id:受信したmessageのID
		//[in] receive_buffer:受信したmessage
		//[in] pub:16進数表記文字列をpublishするインスタンス
		void publishReceiveString(const long message_id, const std::array<uint8_t, 8> &receive_buffer,
			const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub)
		{
			std::stringstream ss;
			ss << std::hex <<message_id << ':';
			for(unsigned int i=0; i<wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT; i++)
			{
				ss << std::hex << std::setw(2) << +receive_buffer[i];
				if(i != wada_vmc_msgs::msg::CanBuffer::DATA_LENGHT-1) ss << ',';
			}
			std_msgs::msg::String str;
			str.data = ss.str();
			pub->publish(str);
		}

	public:
		ReceiveToTopic20221111(const rclcpp::NodeOptions &node_option)
			: rclcpp::Node("wada_vmc_receive_to_topic", node_option)
		{
			this->declare_parameter<double>("steering_can_value_left_to_tire_rad_slope", 0.000926189);
			this->declare_parameter<double>("steering_can_value_left_to_tire_rad_intercept", 0.050007438);
			this->declare_parameter<double>("steering_can_value_right_to_tire_rad_slope", 0.000887424);
			this->declare_parameter<double>("steering_can_value_right_to_tire_rad_intercept", 0.031315874);
			this->declare_parameter<int>("steer_input_center", 1024);
			this->declare_parameter<std::string>("select_can_device", "");

			steering_can_value_left_to_tire_rad_slope_ = get_parameter("steering_can_value_left_to_tire_rad_slope").as_double();
			steering_can_value_left_to_tire_rad_intercept_ = get_parameter("steering_can_value_left_to_tire_rad_intercept").as_double();
			steering_can_value_right_to_tire_rad_slope_ = get_parameter("steering_can_value_right_to_tire_rad_slope").as_double();
			steering_can_value_right_to_tire_rad_intercept_ = get_parameter("steering_can_value_right_to_tire_rad_intercept").as_double();
			steer_input_center_ = static_cast<int16_t>(get_parameter("steer_input_center").as_int());
			select_can_device_ = get_parameter("select_can_device").as_string();

			sub_can_receive_ = this->create_subscription<wada_vmc_msgs::msg::CanBuffer>("can_receive", rclcpp::SensorDataQoS(),
				std::bind(&ReceiveToTopic20221111::callbackCanBuffer, this, std::placeholders::_1));

			pub_debug_receive_100_16_ = this->create_publisher<std_msgs::msg::String>("debug/receive_hex100", rclcpp::QoS(1));
			if(select_can_device_ == "pedal")
			{
				pub501_ = this->create_publisher<wada_vmc_msgs::msg::Can501_20221111>("can501", rclcpp::QoS(1));
				pub_debug_receive_501_16_ = this->create_publisher<std_msgs::msg::String>("debug/receive_hex501", rclcpp::QoS(1));	
			}
			else if(select_can_device_ == "steer")
			{
				pub502_ = this->create_publisher<wada_vmc_msgs::msg::Can502_20221111>("can502", rclcpp::QoS(1));
				pub_steering_status_rad_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("steering_status", rclcpp::QoS(1));
				pub_debug_receive_502_16_ = this->create_publisher<std_msgs::msg::String>("debug/receive_hex502", rclcpp::QoS(1));
			}
		}
	};
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<wada_vmc::ReceiveToTopic20221111> node = std::make_shared<wada_vmc::ReceiveToTopic20221111>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
