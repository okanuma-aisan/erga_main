#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gnss_pose_cov_;
rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_gnss_twist_cov_;
rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_gnss_surfacespeed_;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_gnss_imu_;
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nav_sat_fix_;

class GnssPoseMqttSub : public rclcpp::Node
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
		//rclcpp::Clock ros_clock(RCL_ROS_TIME);
		//rcl_time_point_value_t nowtime_nano = ros_clock.now().nanoseconds();

		GnssPoseMqttSub* node = (GnssPoseMqttSub*)obj;
		rclcpp::Time nowtime = node->now();
		std::string mqtt_topic(message->topic);
		//std::cout << "aaa," << mqtt_topic << std::endl;

		if(mqtt_topic == "/gnss/pose")
		{
			geometry_msgs::msg::PoseWithCovarianceStamped gnss_pose;
			//gnss_pose.header.stamp.sec = nowtime_nano / 1000000000;
			//gnss_pose.header.stamp.nanosec = nowtime_nano % 1000000000;
			gnss_pose.header.stamp = nowtime;
			gnss_pose.header.frame_id = "map";
			gnss_pose.pose.pose.position.x = payload_ptr[0];
			gnss_pose.pose.pose.position.y = payload_ptr[1];
			gnss_pose.pose.pose.position.z = payload_ptr[2];
			gnss_pose.pose.pose.orientation.x = payload_ptr[3];
			gnss_pose.pose.pose.orientation.y = payload_ptr[4];
			gnss_pose.pose.pose.orientation.z = payload_ptr[5];
			gnss_pose.pose.pose.orientation.w = payload_ptr[6];
			for(double &c : gnss_pose.pose.covariance)
				c = 0;
			pub_gnss_pose_cov_->publish(gnss_pose);
			std::cout << "sub : pose" << std::endl;
		}
		else if(mqtt_topic == "/gnss/twist")
		{
			geometry_msgs::msg::TwistWithCovarianceStamped gnss_twist;
			//gnss_twist.header.stamp.sec = nowtime_nano / 1000000000;
			//gnss_twist.header.stamp.nanosec = nowtime_nano % 1000000000;
			gnss_twist.header.stamp = nowtime;
			gnss_twist.header.frame_id = "gnss_base_link";
			gnss_twist.twist.twist.linear.x = payload_ptr[0];
			gnss_twist.twist.twist.linear.y = payload_ptr[1];
			gnss_twist.twist.twist.linear.z = payload_ptr[2];
			gnss_twist.twist.twist.angular.x = payload_ptr[3];
			gnss_twist.twist.twist.angular.y = payload_ptr[4];
			gnss_twist.twist.twist.angular.z = payload_ptr[5];
			for(double &c : gnss_twist.twist.covariance)
				c = 0;
			pub_gnss_twist_cov_->publish(gnss_twist);

			geometry_msgs::msg::TwistWithCovarianceStamped surface_speed;
			surface_speed.header.stamp = nowtime;
			surface_speed.header.frame_id = "/base_link";
			surface_speed.twist.twist.linear.x = std::sqrt(payload_ptr[0]*payload_ptr[0] + payload_ptr[1]*payload_ptr[1] + payload_ptr[2]*payload_ptr[2]);
			pub_gnss_surfacespeed_->publish(surface_speed);
			std::cout << "sub : twist" << std::endl;
		}
		else if(mqtt_topic == "/gnss/imu")
		{
			sensor_msgs::msg::Imu imu;
			imu.header.stamp = nowtime;
			imu.header.frame_id = "/imu_link";
			imu.orientation.x = payload_ptr[0];
			imu.orientation.y = payload_ptr[1];
			imu.orientation.z = payload_ptr[2];
			imu.orientation.w = payload_ptr[3];
			imu.linear_acceleration.x = payload_ptr[4];
			imu.linear_acceleration.y = payload_ptr[5];
			imu.linear_acceleration.z = payload_ptr[6];
			imu.angular_velocity.x = payload_ptr[7];
			imu.angular_velocity.y = payload_ptr[8];
			imu.angular_velocity.z = payload_ptr[9];
			pub_gnss_imu_->publish(imu);
			std::cout << "sub : imu" << std::endl;
		}
		else if(mqtt_topic == "/gnss/direction")
		{
			sensor_msgs::msg::NavSatFix nav;
			nav.header.stamp = nowtime;
			nav.header.frame_id = "map";
			nav.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
			nav.status.service = 15; //not use 2022_09_05 //sensor_msgs::msg::NavSatStatus::SERVICE_GPS | sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS | sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO | sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
			nav.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
			nav.latitude = payload_ptr[0];
			nav.longitude = payload_ptr[1];
			nav.altitude = payload_ptr[2];
			for(double &v : nav.position_covariance) v = 0;
			pub_nav_sat_fix_->publish(nav);
			std::cout << "sub : nav_sat_fix" << std::endl;
		}
	}
public:
	GnssPoseMqttSub() : rclcpp::Node("gnss_pose_mqtt_sub")
	{
   		std::string ip = this->declare_parameter<std::string>("ip", "192.168.1.20");
		pub_gnss_pose_cov_= this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"/sensing/gnss/pose_with_covariance", rclcpp::SensorDataQoS());
		pub_gnss_twist_cov_= this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
			"/sensing/gnss/twist_with_covariance", rclcpp::SensorDataQoS());
		pub_gnss_surfacespeed_= this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
			"/sensing/gnss/novatel_oem7/surface_speed", rclcpp::SensorDataQoS());
		pub_gnss_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(
			"/sensing/gnss/novatel_oem7/imu", rclcpp::SensorDataQoS());
		pub_nav_sat_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
			"/sensing/gnss/novatel_oem7/nav_sat_fix", rclcpp::SensorDataQoS());

		//mosquit初期化
		int mos_ret = mosquitto_lib_init();
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : init" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoインスタンスの作成
		std::string mqtt_node_name = "gnss_pose_sub";
		bool clean_session = true;
		this->mosq_ = mosquitto_new(mqtt_node_name.c_str(), clean_session, (void*)this);

		if(this->mosq_ == nullptr)
		{
			std::cout << "error : mosquitto_new" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoコールバックのセット
		mosquitto_connect_callback_set(this->mosq_, GnssPoseMqttSub::mqttOnConnect);
		mosquitto_disconnect_callback_set(this->mosq_, GnssPoseMqttSub::mqttOnDisconnect);
		mosquitto_message_callback_set(this->mosq_, GnssPoseMqttSub::mqttOnMessage);

		//mosquittoのsubscribe側のコネクト
		mos_ret = mosquitto_connect_bind(this->mosq_, ip.c_str(), 1883, 60, NULL);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : bind   " << mos_ret << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquitto subscribe準備
		std::string mqtt_topic_name_pose = "/gnss/pose";
		mos_ret = mosquitto_subscribe(this->mosq_, NULL, mqtt_topic_name_pose.c_str(), 0);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_subscribe   " << mos_ret << std::endl;
			mosquitto_lib_cleanup();
			return;
		}
		std::string mqtt_topic_name_twist = "/gnss/twist";
		mos_ret = mosquitto_subscribe(this->mosq_, NULL, mqtt_topic_name_twist.c_str(), 0);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_subscribe   " << mos_ret << std::endl;
			mosquitto_lib_cleanup();
			return;
		}
		std::string mqtt_topic_name_imu = "/gnss/imu";
		mos_ret = mosquitto_subscribe(this->mosq_, NULL, mqtt_topic_name_imu.c_str(), 0);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_subscribe   " << mos_ret << std::endl;
			mosquitto_lib_cleanup();
			return;
		}
		/*std::string mqtt_topic_name_direction = "/gnss/direction";
		mos_ret = mosquitto_subscribe(this->mosq_, NULL, mqtt_topic_name_direction.c_str(), 0);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_subscribe   " << mos_ret << std::endl;
			mosquitto_lib_cleanup();
			return;
		}*/

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
	std::shared_ptr<GnssPoseMqttSub> vehicle_status_sub = std::make_shared<GnssPoseMqttSub>();
	rclcpp::spin(vehicle_status_sub);
	rclcpp::shutdown();
	return 0;
}