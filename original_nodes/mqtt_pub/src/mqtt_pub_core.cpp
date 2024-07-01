#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>

class MqttPub : public rclcpp::Node
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

	//BROKEにMESSAGE送信後に実行されるcallback関数
	static void mqttOnPublish(struct mosquitto *mosq, void *userdata, int mid)
	{

	}

	//-------------ros2関連-------------
	rclcpp::TimerBase::SharedPtr timer_control_;//タイマーオブジェクト

	void MqttPub::timerCallback()
	{

	}
public:
	MqttPub()
		: rclcpp::Node("mqtt_pub")
		, mosq_(nullptr)
		, mosq_success_flag_(false)
	{
		//mosquit初期化
		int mos_ret = mosquitto_lib_init();
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : init" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoインスタンスの作成
		std::string mqtt_node_name = "mqtt_pub";
		bool clean_session = true;
		this->mosq_ = mosquitto_new(mqtt_node_name.c_str(), clean_session, NULL);
		if(this->mosq_ == nullptr)
		{
			std::cout << "error : mosquitto_new" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoコールバックのセット
		mosquitto_connect_callback_set(this->mosq_, MqttPub::mqttOnConnect);
		mosquitto_disconnect_callback_set(this->mosq_, MqttPub::mqttOnDisconnect);
		mosquitto_publish_callback_set(this->mosq_, MqttPub::mqttOnPublish);

		auto period_control_ns =
    		std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1));
		timer_control_ = rclcpp::create_timer(
			this, get_clock(), period_control_ns, std::bind(&MqttPub::timerCallback, this));
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<MqttPub> node = std::make_shared<MqttPub>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}