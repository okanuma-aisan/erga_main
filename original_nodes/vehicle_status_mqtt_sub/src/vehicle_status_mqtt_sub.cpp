#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_vehicle_report;

class VehicleStatusMqttSub : public rclcpp::Node
{
private:
	//-------------mqtt関連-------------
	bool mosq_success_flag_;//mosquitto正常起動フラグ
	struct mosquitto *mosq_;//mosquittoインスタンス

	//Brokerとの接続成功時に実行されるcallback関数
	static void mqttOnConnect(struct mosquitto *mosq, void *obj, int result)
	{
	}

	//Brokerとの接続を切断した時に実行されるcallback関数
	static void mqttOnDisconnect(struct mosquitto *mosq, void *obj, int rc)
	{
	}

	//メッセージ受信処理
	static void mqttOnMessage(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
	{
		double* payload_ptr = reinterpret_cast<double*>(message->payload);
		autoware_auto_vehicle_msgs::msg::VelocityReport report;
		rclcpp::Clock ros_clock(RCL_ROS_TIME);
		rcl_time_point_value_t nowtime_nano = ros_clock.now().nanoseconds();
		report.header.stamp.set__nanosec(nowtime_nano);
		//report.header.stamp.sec = nowtime_nano / 1000000000;
		//report.header.stamp.nanosec = nowtime_nano % 1000000000;
		report.header.frame_id = "base_link";
		report.longitudinal_velocity = static_cast<float>(payload_ptr[0]);
		report.lateral_velocity = static_cast<float>(payload_ptr[1]);
		report.heading_rate = static_cast<float>(payload_ptr[2]);
		pub_vehicle_report->publish(report);
	}
public:
	VehicleStatusMqttSub() : rclcpp::Node("vehicle_status_mqtt_sub")
		, mosq_(nullptr)
		, mosq_success_flag_(false)
	{
		std::string ip = this->declare_parameter<std::string>("ip", "192.168.12.5");
		pub_vehicle_report = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
			"/vehicle/status/velocity_status",rclcpp::QoS(10));

		//mosquit初期化
		int mos_ret = mosquitto_lib_init();
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : init" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoインスタンスの作成
		std::string mqtt_node_name = "vehicle_status_sub";
		bool clean_session = true;
		this->mosq_ = mosquitto_new(mqtt_node_name.c_str(), clean_session, (void*)this);

		if(this->mosq_ == nullptr)
		{
			std::cout << "error : mosquitto_new" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoコールバックのセット
		mosquitto_connect_callback_set(this->mosq_, VehicleStatusMqttSub::mqttOnConnect);
		mosquitto_disconnect_callback_set(this->mosq_, VehicleStatusMqttSub::mqttOnDisconnect);
		mosquitto_message_callback_set(this->mosq_, VehicleStatusMqttSub::mqttOnMessage);

		//mosquittoのsubscribe側のコネクト
		mos_ret = mosquitto_connect_bind(this->mosq_, ip.c_str(), 1883, 60, NULL);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : bind   " << mos_ret << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquitto subscribe準備
		std::string mqtt_topic_name = "/vehicle/status";
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

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<VehicleStatusMqttSub> vehicle_status_sub = std::make_shared<VehicleStatusMqttSub>();
	rclcpp::spin(vehicle_status_sub);
	rclcpp::shutdown();
	return 0;
}