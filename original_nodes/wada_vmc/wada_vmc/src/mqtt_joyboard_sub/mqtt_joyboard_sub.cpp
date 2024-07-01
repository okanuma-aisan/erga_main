#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <wada_vmc_msgs/msg/mqtt_joyboard_sub.hpp>

namespace wada_vmc
{
	
	class DataConveyor
	{
	private:
		uint8_t *data_list_;
		size_t byte_cou_;
	public:
		DataConveyor(uint8_t* dl)
			: data_list_(dl)
			, byte_cou_(0)
		{
		}

		template<class T>
		T getVal()
		{
			T* val = reinterpret_cast<T*>(&this->data_list_[this->byte_cou_]);
			this->byte_cou_ += sizeof(T);
			return *val;
		}

		size_t byteCount()
		{
			return this->byte_cou_;
		}
	};

	class MqttJoyboardSub : public rclcpp::Node
	{
	private:
		//-------------mqtt関連-------------
		bool mosq_success_flag_;//mosquitto正常起動フラグ
		struct mosquitto *mosq_;//mosquittoインスタンス

		//Brokerとの接続成功時に実行されるcallback関数
		static void mqttOnConnect(struct mosquitto*, void*, int)
		{
		}

		//Brokerとの接続を切断した時に実行されるcallback関数
		static void mqttOnDisconnect(struct mosquitto*, void*, int)
		{
		}

		//メッセージ受信処理
		static void mqttOnMessage(struct mosquitto*, void*, const struct mosquitto_message *message)
		{
			DataConveyor conveyor(reinterpret_cast<uint8_t*>(message->payload));

			wada_vmc_msgs::msg::MqttJoyboardSub msg;
			msg.linear_velocity = conveyor.getVal<double>();
			msg.steering_angle = conveyor.getVal<double>();
			msg.blinker = conveyor.getVal<int32_t>();
			msg.accel_storke_cap = conveyor.getVal<int16_t>();
			msg.local_way_max_vel_mps = conveyor.getVal<double>();
			msg.lat_std_dev = conveyor.getVal<double>();
			msg.lon_std_dev = conveyor.getVal<double>();
			msg.alt_std_dev = conveyor.getVal<double>();
			msg.baselink_distance = conveyor.getVal<double>();
			msg.baselink_angular = conveyor.getVal<double>();
			msg.localizer_select = conveyor.getVal<int32_t>();
			msg.use_safety_localizer = conveyor.getVal<bool>();
			msg.accel_stroke_cap_temporary_stopper = conveyor.getVal<int16_t>();
			msg.accel_stroke_cap_mobileye = conveyor.getVal<int16_t>();
			/*double* payload_ptr = reinterpret_cast<double*>(message->payload);
			geometry_msgs::msg::TwistWithCovarianceStamped can_oodm;
			rclcpp::Clock ros_clock(RCL_ROS_TIME);
			rcl_time_point_value_t nowtime_nano = ros_clock.now().nanoseconds();
			can_oodm.header.stamp.sec = nowtime_nano / 1000000000;
			can_oodm.header.stamp.nanosec = nowtime_nano % 1000000000;
			can_oodm.header.frame_id = "base_link";
			can_oodm.twist.twist.linear.x = payload_ptr[0];
			can_oodm.twist.twist.linear.y = 0;
			can_oodm.twist.twist.linear.z = 0;
			can_oodm.twist.twist.angular.x = 0;
			can_oodm.twist.twist.angular.y = 0;
			can_oodm.twist.twist.angular.z = payload_ptr[1];
			for(double &c : can_oodm.twist.covariance)
				c = 0;
			pub_can_velocity_->publish(can_oodm);*/
		}
	public:
		MqttJoyboardSub(const rclcpp::NodeOptions &node_option)
			: rclcpp::Node("wada_vmc_calculation", node_option)
		{
			std::string ip = this->declare_parameter<std::string>("ip", "192.168.1.20");

		//mosquit初期化
		int mos_ret = mosquitto_lib_init();
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : init" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoインスタンスの作成
		std::string mqtt_node_name = "mqtt_can_sub";
		bool clean_session = true;
		this->mosq_ = mosquitto_new(mqtt_node_name.c_str(), clean_session, (void*)this);

		if(this->mosq_ == nullptr)
		{
			std::cout << "error : mosquitto_new" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoコールバックのセット
		mosquitto_connect_callback_set(this->mosq_, MqttJoyboardSub::mqttOnConnect);
		mosquitto_disconnect_callback_set(this->mosq_, MqttJoyboardSub::mqttOnDisconnect);
		mosquitto_message_callback_set(this->mosq_, MqttJoyboardSub::mqttOnMessage);

		//mosquittoのsubscribe側のコネクト
		mos_ret = mosquitto_connect_bind(this->mosq_, ip.c_str(), 1883, 60, NULL);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : bind   " << mos_ret << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquitto subscribe準備
		std::string mqtt_topic_name = "/joyboard_data";
		mos_ret = mosquitto_subscribe(this->mosq_, NULL, mqtt_topic_name.c_str(), 0);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_subscribe   " << mos_ret << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquitto subscribeスタート(これはros用で、rosを用いない場合はmosquitto_loop_foreverがある)
		mos_ret = mosquitto_loop_start(this->mosq_);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : loop_start   " << mos_ret << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		std::cout << "mqtt connect success" << std::endl;
		this->mosq_success_flag_ = true;
		}
	};
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<wada_vmc::MqttJoyboardSub> node = std::make_shared<wada_vmc::MqttJoyboardSub>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}