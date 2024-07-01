#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

//joyボード関連include
#include <wada_vmc_msgs/msg/can_buffer.hpp>
#include <wada_vmc_msgs/msg/can501_20210103.hpp>
#include <wada_vmc_msgs/msg/can502_20210103.hpp>
#include <wada_vmc_msgs/msg/can503_20210103.hpp>

//autoware関連include
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

namespace wada_vmc
{
	class ReceiveToTopic20210103 : public rclcpp::Node
	{
	private://ros2 subscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::CanBuffer>::SharedPtr sub_can_receive_;//canメッセージのsubscriber

	private://ros2 publisher(joyボード関連)
		rclcpp::Publisher<wada_vmc_msgs::msg::Can501_20210103>::SharedPtr pub501_;//joyボード受信データ0x501のpulisher
		rclcpp::Publisher<wada_vmc_msgs::msg::Can502_20210103>::SharedPtr pub502_;//joyボード受信データ0x502のpulisher
		rclcpp::Publisher<wada_vmc_msgs::msg::Can503_20210103>::SharedPtr pub503_;//joyボード受信データ0x503のpulisher
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_receive_100_16_;//生のjoyボード受信データ(0x100)を16進数表記で配信するpublisher
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_receive_501_16_;//生のjoyボード受信データ(0x501)を16進数表記で配信するpublisher
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_receive_502_16_;//生のjoyボード受信データ(0x502)を16進数表記で配信するpublisher
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_receive_503_16_;//生のjoyボード受信データ(0x503)を16進数表記で配信するpublisher
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_steer_auto_mode_;//ステアがAutoModeか？
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_pedal_auto_mode_;//ペダルがAutoModeか？

	private://ros2 publisher(autoware関連)
		rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steering_status_rad_;//現在のホイール角度のpublisher 単位はrad autowareのcontrol系の処理に渡す
		rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_velocity_status_;//現在の速度のpublisher 単位はm/s autowareのcontrol系の処理に渡す

	private://車両固有情報
		double steering_can_value_left_to_tire_rad_slope_;
		double steering_can_value_right_to_tire_rad_slope_;
		double steering_can_value_left_to_tire_rad_intercept_;
		double steering_can_value_right_to_tire_rad_intercept_;
		double wheelrad_to_steering_can_value_left_;
		double wheelrad_to_steering_can_value_right_;
		int16_t steer_input_correction_receive_;////joyボードのステア角度補正値(送信側)
		double velocity_magn_;//速度補正値

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
				publish501(can_receive->receive_data);
				publishReceiveString(can_receive->message_id, can_receive->receive_data, pub_debug_receive_501_16_);
				break;
			case 0x503:
				//if(select_can_device_ == "pedal" || select_can_device_ == "all")
				{
					publish503(can_receive->receive_data);
					publishReceiveString(can_receive->message_id, can_receive->receive_data, pub_debug_receive_503_16_);
				}
				break;
			case 0x502:
				//if(select_can_device_ == "steer" || select_can_device_ == "all")
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
			wada_vmc_msgs::msg::Can501_20210103 can501;//コンポーネント実行時にトピック送受信でのメモリコピーを回避するために、UniquePtrを使用
			can501.stamp = this->now();

			//ジョイボード起動時、または通信が一定時間行われなかった場合に、0番目は55になる
			can501.first_lock = receive_buffer[0] == 0x55;

			//steer auto アンサーバック
			can501.steer_auto_answer_back = (receive_buffer[0] & 0xF0) >> 4;

			//pedal auto アンサーバック
			can501.pedal_auto_answer_back = receive_buffer[0] & 0x0F;

			//steer autoフラグ 0はV0 10はAUTO それ以外はエラー
			can501.steer_mode = (receive_buffer[1] & 0xF0) >> 4;

			//drive autoフラグ 0はV0 10はAUTO それ以外はエラー
			can501.drive_mode = receive_buffer[1] & 0x0F;

			//steering angle入力値の返答値
			unsigned char *str_tmp = (unsigned char*)&can501.steer_reply;
			str_tmp[0] = receive_buffer[3];  str_tmp[1] = receive_buffer[2];

			//stroke入力値の返答値
			unsigned char *vel_tmp = (unsigned char*)&can501.pedal_reply;
			vel_tmp[0] = receive_buffer[5];  vel_tmp[1] = receive_buffer[4];

			//緊急ブレーキ
			if(receive_buffer[6] & 0x80) can501.emergency_brake = wada_vmc_msgs::msg::Can501_20210103::EMERGENCY_BRAKE_ON;
			else if(receive_buffer[6] & 0x40) can501.emergency_brake = wada_vmc_msgs::msg::Can501_20210103::EMERGENCY_BRAKE_ERROR;
			else can501.emergency_brake = wada_vmc_msgs::msg::Can501_20210103::EMERGENCY_BRAKE_OFF;

			//エンジン始動
			can501.engine_start = (receive_buffer[6] & 0x20) ? true : false;

			//イグニション
			can501.ignition = (receive_buffer[6] & 0x10) ? true : false;

			//シフトボタン化状態
			can501.shift_bottom = (receive_buffer[6] & 0x08) ? true : false;

			//アクセル介入判定
			can501.accel_intervention = (receive_buffer[6] & 0x04) ? true : false;

			//右ウィンカー点滅状態
			can501.right_blinker = (receive_buffer[6] & 0x02) ? true : false;

			//左ウィンカー点滅状態
			can501.left_blinker = (receive_buffer[6] & 0x01) ? true : false;

			//log記録開始フラグ(ステア・ペダル・シフト・ウインカーメイン監視)
			can501.log_start = (receive_buffer[7] & 0x20) ? true : false;

			//自動運転可否(すす燃焼状態(DBS)他)
			can501.auto_drive_ok = (receive_buffer[7] & 0x10) ? true : false;

			//シフトボタンD押下(DIN0=1&DIN7=1)
			can501.shift_d = (receive_buffer[7] & 0x08) ? true : false;

			//シフトボタンN押下(DIN0=1&DIN7=1)
			can501.shift_n = (receive_buffer[7] & 0x04) ? true : false;

			//シフトボタンR押下(DIN0=1&DIN7=1)
			can501.shift_r = (receive_buffer[7] & 0x02) ? true : false;

			//シフト自動化状態(DIN7=1) 決定 (DIN7=0)
			can501.shift_auto = (receive_buffer[7] & 0x01) ? true : false;

			//クルーズコントロール速度+
			can501.curse_velocity_plus = can501.shift_d;

			//クルーズコントロール速度-
			can501.curse_velocity_minus = can501.shift_n;

			//車間距離変更
			can501.distance_between_two_cars = can501.shift_r;

			pub501_->publish(can501);
		}

		//message ID 502をpublish
		//[in] receive_buffer:受信したmessage
		void publish502(const std::array<uint8_t, 8> &receive_buffer)
		{
			rclcpp::Time rosnowtime = this->now();

			wada_vmc_msgs::msg::Can502_20210103 can502;//コンポーネント実行時にトピック送受信でのメモリコピーを回避するために、UniquePtrを使用
			can502.stamp = rosnowtime;

			//true:自動運転モード false:マニュアルモード
			can502.auto_mode = (receive_buffer[0] & 0x40) ? true : false;

			//パルスモード  １：Ｐ１，２：Ｐ２，４：Ｐ４，８：Ｐ８，０：Ｐ１６
			can502.pulse_mode = (receive_buffer[0] & 0x3c) >> 2;

			//書き込み許可線ON
			can502.write_permission_on = (receive_buffer[0] & 0x02) ? true : false;

			//温度異常
			can502.temperature_anomaly = (receive_buffer[0] & 0x01) ? true : false;

			//過電流
			can502.over_current = (receive_buffer[1] & 0x80) ? true : false;

			//クラッチスイッチ状態
			can502.clutch = (receive_buffer[1] & 0x40) ? false : true;

			//ブレーキロックセット(ハンドルでは未使用、常に０)
			can502.brake_lock_set = (receive_buffer[1] & 0x20) ? true : false;

			//モーター線断線
			can502.motor_disconnection = (receive_buffer[1] & 0x10) ? true : false;

			//電源電圧低
			can502.power_supply_voltage_drop = (receive_buffer[1] & 0x08) ? true : false;

			//クラッチ線断線
			can502.clutch_disconnection = (receive_buffer[1] & 0x04) ? true : false;

			//メカポテンション断線
			can502.mecha_potentiometer_disconnection = (receive_buffer[1] & 0x02) ? true : false;

			//ジョイスティック断線
			can502.joy_disconnection = (receive_buffer[1] & 0x01) ? true : false;

			//ステアリング指令電圧データ  データ　0-2048　が0-5Vに対応
			uint8_t *uint8tmp = reinterpret_cast<uint8_t*>(&can502.handle_pot_average);
			uint8tmp[0] = receive_buffer[3]; uint8tmp[1] = receive_buffer[2];
			can502.handle_pot_average_v = static_cast<float>(can502.handle_pot_average * 5.0 / 2048.0);

			//ステアリング角度  ＊２０　deg 表記
			uint8tmp = reinterpret_cast<uint8_t*>(&can502.steering_angle);
			uint8tmp[0] = receive_buffer[5]; uint8tmp[1] = receive_buffer[4];

			//ステア補正
			can502.steering_angle += steer_input_correction_receive_;
			
			//ステアリング角度
			can502.steering_angle_deg = static_cast<float>(can502.steering_angle / 20.0);

			//タイヤ角
			if(can502.steering_angle > 0)//タイヤ角度の計算
			{
				can502.tire_angle_rad = (can502.steering_angle) * 
					steering_can_value_left_to_tire_rad_slope_ + steering_can_value_left_to_tire_rad_intercept_;
				//RCLCPP_INFO(this->get_logger(), "left_angle,%lf", can502.tire_angle_rad);
			}
			else
			{
				can502.tire_angle_rad = (can502.steering_angle) * 
					steering_can_value_right_to_tire_rad_slope_ + steering_can_value_right_to_tire_rad_intercept_;
				//RCLCPP_INFO(this->get_logger(), "right_angle,%lf", can502.tire_angle_rad);
			}

			//車両速度  km/h*100表記
			uint8tmp = reinterpret_cast<uint8_t*>(&can502.velocity);
			uint8tmp[0] = receive_buffer[7]; uint8tmp[1] = receive_buffer[6];
			can502.velocity_kmh = static_cast<float>(can502.velocity / 100.0 * velocity_magn_);

			pub502_->publish(can502);

			//MPCに渡すステア情報をpublish
			autoware_auto_vehicle_msgs::msg::SteeringReport steering_report;
			steering_report.stamp = rosnowtime;
			steering_report.steering_tire_angle = can502.tire_angle_rad;
			pub_steering_status_rad_->publish(steering_report);

			autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report;
			velocity_report.header.stamp = rosnowtime;
			velocity_report.header.frame_id = "base_link";
			velocity_report.longitudinal_velocity = can502.velocity_kmh / 3.6;
			velocity_report.lateral_velocity = 0;
			velocity_report.heading_rate = 0;
			pub_velocity_status_->publish(velocity_report);

			//ステアの操作モードをpublish
			std_msgs::msg::Bool msg_steer_auto;
			msg_steer_auto.data = can502.auto_mode;
			pub_steer_auto_mode_->publish(msg_steer_auto);
		}

		//message ID 503をpublish
		//[in] receive_buffer:受信したmessage
		void publish503(const std::array<uint8_t, 8> &receive_buffer)
		{
			wada_vmc_msgs::msg::Can503_20210103 can503;//コンポーネント実行時にトピック送受信でのメモリコピーを回避するために、UniquePtrを使用
			can503.stamp = this->now();

			//true:自動運転モード false:マニュアルモード
			can503.auto_mode = (receive_buffer[0] & 0x80) ? true : false;

			//パルスモード  １：Ｐ１，２：Ｐ２，４：Ｐ４，８：Ｐ８，０：Ｐ１６
			can503.pulse_mode = (receive_buffer[0] & 0x78) >> 3;

			//空気圧異常
			can503.abnormal_air_pressure = (receive_buffer[0] & 0x04) ? true : false;

			//書き込み許可線ON
			can503.write_permission_on = (receive_buffer[0] & 0x02) ? true : false;

			//温度異常
			can503.temperature_anomaly = (receive_buffer[0] & 0x01) ? true : false;

			//過電流
			can503.over_current = (receive_buffer[1] & 0x80) ? true : false;

			//クラッチスイッチ状態
			can503.clutch = (receive_buffer[1] & 0x40) ? false : true;

			//ブレーキロックセット(ハンドルでは未使用、常に０)
			can503.brake_lock_set = (receive_buffer[1] & 0x20) ? true : false;

			//モーター線断線
			can503.motor_disconnection = (receive_buffer[1] & 0x10) ? true : false;

			//電源電圧低
			can503.power_supply_voltage_drop = (receive_buffer[1] & 0x08) ? true : false;

			//クラッチ線断線
			can503.clutch_disconnection = (receive_buffer[1] & 0x04) ? true : false;

			//メカポテンション断線
			can503.mecha_potentiometer_disconnection = (receive_buffer[1] & 0x02) ? true : false;

			//ジョイスティック断線
			can503.joy_disconnection = (receive_buffer[1] & 0x01) ? true : false;

			//ペダル指令電圧データ  データ　0-2048　が0-5Vに対応
			uint8_t *uint8tmp = reinterpret_cast<uint8_t*>(&can503.pedal_command_voltage);
			uint8tmp[0] = receive_buffer[3]; uint8tmp[1] = receive_buffer[2];
			can503.pedal_command_voltage_v = static_cast<float>(can503.pedal_command_voltage * 5.0 / 2048.0);

			//ペダル変位量　ペダル計測用ポテンショメータの測定値とする２０２０．４．９
			uint8tmp = reinterpret_cast<uint8_t*>(&can503.pedal_displacement);
			uint8tmp[0] = receive_buffer[5]; uint8tmp[1] = receive_buffer[4];

			//エンジン回転数 RPM表記
			uint8tmp = reinterpret_cast<uint8_t*>(&can503.engine_rpm);
			uint8tmp[0] = receive_buffer[7]; uint8tmp[1] = receive_buffer[6];

			pub503_->publish(can503);

			//ペダルの操作モードをpublish
			std_msgs::msg::Bool msg_pedal_auto;
			msg_pedal_auto.data = can503.auto_mode;
			pub_pedal_auto_mode_->publish(msg_pedal_auto);
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
		ReceiveToTopic20210103(const rclcpp::NodeOptions &node_option)
			: rclcpp::Node("receive_to_topic", node_option)
		{
			this->declare_parameter<double>("steering_can_value_left_to_tire_rad_slope", 3.99845295833998E-05);
			this->declare_parameter<double>("steering_can_value_left_to_tire_rad_intercept", 0.0);
			this->declare_parameter<double>("steering_can_value_right_to_tire_rad_slope", 3.73609509281213E-05);
			this->declare_parameter<double>("steering_can_value_right_to_tire_rad_intercept", 0.0);
			this->declare_parameter<double>("wheelrad_to_steering_can_value_left", 25009.6727514125);
			this->declare_parameter<double>("wheelrad_to_steering_can_value_right", 26765.9140133745);
			this->declare_parameter<int>("steer_input_correction_receive", 0);
			this->declare_parameter<double>("velocity_magn", 1.0);
			this->declare_parameter<std::string>("select_can_device", "");

			steering_can_value_left_to_tire_rad_slope_ = get_parameter("steering_can_value_left_to_tire_rad_slope").as_double();
			steering_can_value_left_to_tire_rad_intercept_ = get_parameter("steering_can_value_left_to_tire_rad_intercept").as_double();
			steering_can_value_right_to_tire_rad_slope_ = get_parameter("steering_can_value_right_to_tire_rad_slope").as_double();
			steering_can_value_right_to_tire_rad_intercept_ = get_parameter("steering_can_value_right_to_tire_rad_intercept").as_double();
			wheelrad_to_steering_can_value_left_ = get_parameter("wheelrad_to_steering_can_value_left").as_double();
			wheelrad_to_steering_can_value_right_ = get_parameter("wheelrad_to_steering_can_value_right").as_double();
			steer_input_correction_receive_ = static_cast<int16_t>(get_parameter("steer_input_correction_receive").as_int());
			velocity_magn_ = get_parameter("velocity_magn").as_double();
			select_can_device_ = get_parameter("select_can_device").as_string();

			RCLCPP_INFO(get_logger(), "steer_input_correction_receive,%d", steer_input_correction_receive_);
			
			sub_can_receive_ = this->create_subscription<wada_vmc_msgs::msg::CanBuffer>("can_receive", rclcpp::SensorDataQoS(),
				std::bind(&ReceiveToTopic20210103::callbackCanBuffer, this, std::placeholders::_1));

			pub_debug_receive_100_16_ = this->create_publisher<std_msgs::msg::String>("debug/receive_hex100", rclcpp::QoS(1));
			pub501_ = this->create_publisher<wada_vmc_msgs::msg::Can501_20210103>("can501", rclcpp::QoS(1));
			pub_debug_receive_501_16_ = this->create_publisher<std_msgs::msg::String>("debug/receive_hex501", rclcpp::QoS(1));

			if(select_can_device_ == "pedal")
			{
				pub503_ = this->create_publisher<wada_vmc_msgs::msg::Can503_20210103>("can503", rclcpp::QoS(1));
				pub_velocity_status_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("velocity_status", rclcpp::QoS(1));
				pub_debug_receive_503_16_ = this->create_publisher<std_msgs::msg::String>("debug/receive_hex503", rclcpp::QoS(1));
				pub_pedal_auto_mode_ = this->create_publisher<std_msgs::msg::Bool>("pedal_auto", rclcpp::QoS(1));
			}
			else if(select_can_device_ == "steer")
			{
				pub502_ = this->create_publisher<wada_vmc_msgs::msg::Can502_20210103>("can502", rclcpp::QoS(1));
				pub_steering_status_rad_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("steering_status", rclcpp::QoS(1));
				pub_debug_receive_502_16_ = this->create_publisher<std_msgs::msg::String>("debug/receive_hex502", rclcpp::QoS(1));
				pub_steer_auto_mode_ = this->create_publisher<std_msgs::msg::Bool>("steer_auto", rclcpp::QoS(1));
			}
			else if(select_can_device_ == "all")
			{
				pub503_ = this->create_publisher<wada_vmc_msgs::msg::Can503_20210103>("can503", rclcpp::QoS(1));
				pub_debug_receive_503_16_ = this->create_publisher<std_msgs::msg::String>("debug/receive_hex503", rclcpp::QoS(1));
				pub502_ = this->create_publisher<wada_vmc_msgs::msg::Can502_20210103>("can502", rclcpp::QoS(1));
				pub_steering_status_rad_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("steering_status", rclcpp::QoS(1));
				pub_velocity_status_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("velocity_status", rclcpp::QoS(1));
				pub_debug_receive_502_16_ = this->create_publisher<std_msgs::msg::String>("debug/receive_hex502", rclcpp::QoS(1));
				pub_pedal_auto_mode_ = this->create_publisher<std_msgs::msg::Bool>("pedal_auto", rclcpp::QoS(1));
				pub_steer_auto_mode_ = this->create_publisher<std_msgs::msg::Bool>("steer_auto", rclcpp::QoS(1));
			}
		}
	};
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<wada_vmc::ReceiveToTopic20210103> node = std::make_shared<wada_vmc::ReceiveToTopic20210103>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

