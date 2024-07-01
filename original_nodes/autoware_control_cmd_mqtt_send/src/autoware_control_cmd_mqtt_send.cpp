#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

//bufにdataをbyte単位でpushする。bufの最初はデータ全長なので、bufの最初にpushしたバイト数を足す
template<class T>
void dataPush(std::vector<uint8_t> &buf, const T data)
{
	const uint8_t* dataptr = (uint8_t*)&data;
	for(size_t i=0; i<sizeof(T); i++) buf.push_back(dataptr[i]);
	size_t* count = reinterpret_cast<size_t*>(buf.data());
	*count += sizeof(T);
}

class CmdMqttPub : public rclcpp::Node
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
	rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_cmd_;//autowareからのcmd

	void callbackCmd(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
	{
		std::vector<uint8_t> pubdata;
		dataPush<float>(pubdata, msg->lateral.steering_tire_angle);
		dataPush<float>(pubdata, msg->lateral.steering_tire_rotation_rate);
		dataPush<float>(pubdata, msg->longitudinal.speed);
		dataPush<float>(pubdata, msg->longitudinal.acceleration);
		dataPush<float>(pubdata, msg->longitudinal.jerk);

		int mos_ret = mosquitto_publish(this->mosq_, nullptr, "/autoware_cmd", pubdata.size(), pubdata.data(), 0, false);
		if(mos_ret == MOSQ_ERR_SUCCESS) std::cout << "mosquitto pub success" << std::endl;
		else std::cout << "mosquitto pub failure" << std::endl;
	}
public:
	CmdMqttPub()
		: rclcpp::Node("cmd_mqtt_pub")
		, mosq_(nullptr)
		, mosq_success_flag_(false)
	{
		std::string ip = this->declare_parameter<std::string>("ip", "192.168.12.4");

		//mosquit初期化
		int mos_ret = mosquitto_lib_init();
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : init" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoインスタンスの作成
		std::string mqtt_node_name = "autoware_control_cmd_mqtt_pub";
		bool clean_session = true;
		this->mosq_ = mosquitto_new(mqtt_node_name.c_str(), clean_session, NULL);
		if(this->mosq_ == nullptr)
		{
			std::cout << "error : mosquitto_new" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoコールバックのセット
		mosquitto_connect_callback_set(this->mosq_, CmdMqttPub::mqttOnConnect);
		mosquitto_disconnect_callback_set(this->mosq_, CmdMqttPub::mqttOnDisconnect);
		mosquitto_publish_callback_set(this->mosq_, CmdMqttPub::mqttOnPublish);

		//mqttサーバーにコネクト
		mos_ret = mosquitto_connect(this->mosq_, ip.c_str(), 1883, 60);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_connect" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		mosq_success_flag_ = true;

		sub_cmd_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
			"/control/command/control_cmd", rclcpp::QoS{1}, std::bind(&CmdMqttPub::callbackCmd, this, std::placeholders::_1));

	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<CmdMqttPub> node = std::make_shared<CmdMqttPub>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}