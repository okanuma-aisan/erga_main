#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <std_msgs/msg/string.hpp>

//ros publisher
//mosquittoのコールバックがstaitcなので、グローバル変数にしないといけない
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_notification;//遠隔から車両へ送られるmqttメッセージをpublish

class TokaiRikaMqttSub : public rclcpp::Node
{
private://mosquitto変数
	bool init_flag_;//mqttの初期化に成功したらtrue
	struct mosquitto *mosq_;//mqttオブジェクト

private://mqtt sub用変数
	std::string car_id_;//mqttで送信する車両ID

private://mqtt callback
	/**
	 * @fn void MqttConnection::callback_on_connect(struct mosquitto *mosq, void *obj, int result)
	 * @brief Brokerとの接続成功時に実行されるcallback関数
	 */
	static void callback_on_connect(struct mosquitto *mosq, void *obj, int result)
	{
		
	}

	/**
	 * @fn void MqttConnection::callback_on_disconnect(struct mosquitto *mosq, void *obj, int rc)
	 * @brief Brokerとの接続を切断した時に実行されるcallback関数
	 */
	static void callback_on_disconnect(struct mosquitto *mosq, void *obj, int rc)
	{

	}

	/**
	 * @fn void MqttConnection::callback_on_publish(struct mosquitto *mosq, void *userdata, int mid)
	 * @brief BrokerにMQTTメッセージ送信後に実行されるcallback関数
	 */
	static void callback_on_publish(struct mosquitto *mosq, void *userdata, int mid)
	{
	}

	/**
	 * @fn void MqttConnection::callback_log(struct mosquitto *mosq, void *userdata, int level, const char *str)
	 * @brief イベントログ出力処理
	 */
	static void callback_log(struct mosquitto *mosq, void *userdata, int level, const char *str)
	{

	}

	/**
	 * @fn void MqttConnection::callback_on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *_message)
	 * @brief MQTTから指定のトピックが配信された際の処理
	 */
	static void callback_on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *_message)
	{
		char mes[1000];
		memcpy(mes, _message->payload, _message->payloadlen);
		mes[_message->payloadlen] = '\0';
		std_msgs::msg::String str;
		str.data = mes;
		pub_notification->publish(str);
	}

private://その他
	//Mosquittoの初期化
	void initMosquitto()
	{
		this->declare_parameter<std::string>("car_id", "0001");
		this->declare_parameter<std::string>("mqtt_host", "a1sjxnq4b8uinx-ats.iot.ap-northeast-1.amazonaws.com");
		this->declare_parameter<std::string>("tls_ca", "/home/sit/load_data/tokai_rika/AmazonRootCA1.pem");
		this->declare_parameter<std::string>("tls_crt", "/home/sit/load_data/tokai_rika/35568e97afba5f1cf52a78e2aadea54140909faf2a41096fbab66e29ca1cd411-certificate.pem.crt");
		this->declare_parameter<std::string>("tls_key", "/home/sit/load_data/tokai_rika/35568e97afba5f1cf52a78e2aadea54140909faf2a41096fbab66e29ca1cd411-private.pem.key");

		car_id_ =  this->get_parameter("car_id").as_string();
		std::string mqtt_host = this->get_parameter("mqtt_host").as_string();
		std::string ca_file = this->get_parameter("tls_ca").as_string();
		std::string crt_file = this->get_parameter("tls_crt").as_string();
		std::string key_file = this->get_parameter("tls_key").as_string();

		//mqtt初期化
		int mqtt_ret = mosquitto_lib_init();
		if(mqtt_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_lib_init" << std::endl;
			return;
		}

		//mqttインスタンスの作成
		std::string mqtt_node_name = "mqtt_tokai_rika_sub";
		bool  clean_session = true;
		mosq_ = mosquitto_new(mqtt_node_name.c_str(), clean_session, nullptr);
		if(mosq_ == nullptr)
		{
			std::cout << "error : mosquitto_new" << std::endl;
			return;
		}

		//mqttコールバックのセット
		mosquitto_connect_callback_set(mosq_, TokaiRikaMqttSub::callback_on_connect);
		mosquitto_disconnect_callback_set(mosq_, TokaiRikaMqttSub::callback_on_disconnect);
		mosquitto_publish_callback_set(mosq_, TokaiRikaMqttSub::callback_on_publish);
		mosquitto_message_callback_set(mosq_, TokaiRikaMqttSub::callback_on_message);
		mosquitto_log_callback_set(mosq_, TokaiRikaMqttSub::callback_log);

		//IDとPASSを設定
		/*mqtt_ret = mosquitto_username_pw_set(this->mosq_, "adv", "saikodcm");
		if(mqtt_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_username_pw_set" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}*/

		/*mqtt_ret = mosquitto_tls_opts_set(mosq_, 1, NULL, NULL);
		if(mqtt_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_tls_opts_set" << std::endl;
			return;
		}*/

		//tls設定
		mqtt_ret = mosquitto_tls_set(mosq_, ca_file.c_str(), NULL, crt_file.c_str(), key_file.c_str(), NULL);
		if(mqtt_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_tls_set" << std::endl;
			return;
		}

		//mqttサーバーにコネクト
		std::cout << "connect start" << std::endl;
		//mqtt_ret = mosquitto_connect(mosq_, "10.193.77.1", 1883, 60);
		//mqtt_ret = mosquitto_connect(mosq_, "localhost", 1883, 60);
		//mqtt_ret = mosquitto_connect(mosq_, mqtt_host.c_str(), 8883, 60);
		mqtt_ret = mosquitto_connect_bind(mosq_, mqtt_host.c_str(), 8883, 60, nullptr);
		if(mqtt_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_connect" << std::endl;
			return;
		}
		std::cout << "connected" << std::endl;
		std::cout << "host_id: " << mqtt_host.c_str() << std::endl;

		mqtt_ret = mosquitto_loop_start(this->mosq_);
		if(mqtt_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_loop_start" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		init_flag_ = true;
	}

	void initRos2()
	{
		std::string topic_notification = this->declare_parameter<std::string>("topic_notification", "/tokai_rika/notification");
		pub_notification = this->create_publisher<std_msgs::msg::String>(topic_notification, rclcpp::QoS(1));
	}

public:
	TokaiRikaMqttSub(const rclcpp::NodeOptions &node_options)
		: rclcpp::Node("tokairika_mqtt_sub", node_options)
		, init_flag_(false)
		, mosq_(nullptr)
	{
		initMosquitto();
		initRos2();
	}

	~TokaiRikaMqttSub()
	{
		pub_notification.reset();//メンバ変数ではないので、明示的にリソースを開放する。
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	std::shared_ptr<TokaiRikaMqttSub> node = std::make_shared<TokaiRikaMqttSub>(options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
