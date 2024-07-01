#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

//mqttで受信したvehicle statusをros2トピックで送信するためのpublisher
	rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;

class CanMqttSub : public rclcpp::Node
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
		std::cout << "data len," << message->payloadlen << std::endl;
		double* payload_ptr = reinterpret_cast<double*>(message->payload);
		//int32_t sec = static_cast<int32_t>(payload_ptr[0]);
		//int32_t nsec = static_cast<int32_t>(payload_ptr[1]);
		rclcpp::Clock ros_clock(RCL_ROS_TIME);
		rcl_time_point_value_t nowtime_nano = ros_clock.now().nanoseconds();
		uint32_t sec = nowtime_nano / 1000000000;
		uint32_t nsec = nowtime_nano % 1000000000;
		double velocity_mps = payload_ptr[2];
		double angle_rad = payload_ptr[3];
		const int covariance_count = 36;
		double covariance[covariance_count];
		for(int i=0; i<covariance_count; i++) covariance[i] = payload_ptr[4+i];

		std::cout << std::fixed << std::setprecision(3) << "stamp," << sec << ":" << nsec << "  speed," << velocity_mps * 3.6 << "  angle," << angle_rad*180/M_PI << std::endl;
		
		geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance;
		twist_with_covariance.header.stamp.sec = static_cast<uint32_t>(sec);
		twist_with_covariance.header.stamp.nanosec = static_cast<uint32_t>(nsec);
		twist_with_covariance.header.frame_id = "";
		twist_with_covariance.twist.twist.linear.x = velocity_mps;
		twist_with_covariance.twist.twist.linear.y = 0;
		twist_with_covariance.twist.twist.linear.z = 0;
		twist_with_covariance.twist.twist.angular.x = 0;
		twist_with_covariance.twist.twist.angular.y = 0;
		twist_with_covariance.twist.twist.angular.z = angle_rad;
		for(int i=0; i<covariance_count; i++) twist_with_covariance.twist.covariance[i] = covariance[i];
		pub_twist_cov_->publish(twist_with_covariance);

		//char* buf = reinterpret_cast<char*>(message->payload);
		//std::cout << buf << std::endl;

		/*if(is_debug) {
			printf("%s(%d)\n", __FUNCTION__, __LINE__);
		}*/

		/*if(message->payloadlen){
			printf("%s ", message->topic);
			fwrite(message->payload, 1, message->payloadlen, stdout);
			printf("\n");
		}else{
			printf("%s (null)\n", message->topic);
		}
		fflush(stdout);*/
	}

	//-------------ros2関連-------------
public:
	CanMqttSub();

	~CanMqttSub()
	{
		mosquitto_destroy(mosq_);
		mosquitto_lib_cleanup();
	}
};

CanMqttSub::CanMqttSub()
	: rclcpp::Node("can_mqtt_sub")
	, mosq_(nullptr)
	, mosq_success_flag_(false)
{
	pub_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
		"/can/twist_cov", rclcpp::QoS{10});

	//mosquit初期化
	int mos_ret = mosquitto_lib_init();
	if(mos_ret != MOSQ_ERR_SUCCESS)
	{
		std::cout << "error : init" << std::endl;
		mosquitto_lib_cleanup();
		return;
	}

	//mosquittoインスタンスの作成
	std::string mqtt_node_name = "can_mqtt_sub";
	bool clean_session = true;
	this->mosq_ = mosquitto_new(mqtt_node_name.c_str(), clean_session, nullptr);

	if(this->mosq_ == nullptr)
	{
		std::cout << "error : mosquitto_new" << std::endl;
		mosquitto_lib_cleanup();
		return;
	}

	//mosquittoコールバックのセット
	mosquitto_connect_callback_set(this->mosq_, CanMqttSub::mqttOnConnect);
	mosquitto_disconnect_callback_set(this->mosq_, CanMqttSub::mqttOnDisconnect);
	mosquitto_message_callback_set(this->mosq_, CanMqttSub::mqttOnMessage);

	//mosquittoのsubscribe側のコネクト
	mos_ret = mosquitto_connect_bind(this->mosq_, "192.168.1.20", 1883, 60, NULL);
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

	this->mosq_success_flag_ = true;
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<CanMqttSub> node = std::make_shared<CanMqttSub>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}