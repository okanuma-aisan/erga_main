#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <orig_system_msgs/msg/date.hpp>
#include <wada_vmc_msgs/msg/can501_20221111.hpp>
#include <wada_vmc_msgs/msg/can502_20221111.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>

struct LatLon//現在車両位置から作成する緯度経度情報
{
	double car_yaw_;//車両のyaw角
	double lat_;//車両緯度
	double lat1m_;
	double lon_;//車両経度
	double lon1m_;
};

class TokaiRikaMqttPub : public rclcpp::Node
{
private://mosquitto変数
	bool init_flag_;//mqttの初期化に成功したらtrue
	struct mosquitto *mosq_;//mqttオブジェクト

private://ros subscriber
	rclcpp::Subscription<orig_system_msgs::msg::Date>::SharedPtr sub_date_;//GNSSから送信される時間(/gnss_time)
	rclcpp::Subscription<wada_vmc_msgs::msg::Can501_20221111>::SharedPtr sub_can501_;//joy boardのペダル情報
	rclcpp::Subscription<wada_vmc_msgs::msg::Can502_20221111>::SharedPtr sub_can502_;//joy boardのハンドル情報
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_curr_pos_;//現在の車両位置
	rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_curr_vel_;//現在の車両速度
	rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr sub_blinker_;//ウィンカー情報

private://ros timer
	rclcpp::TimerBase::SharedPtr timer_;//mqtt送信用タイマー

private://mqtt pub用変数
	std::string car_id_;//mqttで送信する車両ID
	orig_system_msgs::msg::Date date_;//GNSSから送信される時間(/gnss_time)
	wada_vmc_msgs::msg::Can501_20221111 can501_;//joy boardのペダル情報
	wada_vmc_msgs::msg::Can502_20221111 can502_;//joy boardのハンドル情報
	LatLon latlon_;//緯度経度
	geometry_msgs::msg::TwistWithCovarianceStamped curr_vel_;//現在車両速度
	uint8_t blinker_;//ウィンカー情報

private://MGRS用変数
	std::string mgrs_utm_zone_;//MGRSのUTMゾーン
	std::string mgrs_lat_band_;//MGRSの緯度バンド
	std::string mgrs_lattice_id_;//MGESの100,000 メートル格子ID

private://mqtt callback
	/**
	 * @fn void MqttConnection::callback_on_connect(struct mosquitto *mosq, void *obj, int result)
	 * @brief Brokerとの接続成功時に実行されるcallback関数
	 */
	static void callback_on_connect(struct mosquitto *mosq, void *obj, int result)
	{
		std::cout << "connect callback" << std::endl;
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
	 * @fn void MqttConnection::callback_on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *_message)
	 * @brief MQTTから指定のトピックが配信された際の処理
	 */
	static void callback_on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *_message)
	{
		
	}

	/**
	 * @fn void MqttConnection::callback_log(struct mosquitto *mosq, void *userdata, int level, const char *str)
	 * @brief イベントログ出力処理
	 */
	static void callback_log(struct mosquitto *mosq, void *userdata, int level, const char *str)
	{

	}

private://ros callback
	void callbackDate(const orig_system_msgs::msg::Date::SharedPtr date)
	{
		date_ = *date;
	}

	void callbackCan502(const wada_vmc_msgs::msg::Can502_20221111::SharedPtr can502)
	{
		can502_ = *can502;
	}

	void callbackCan501(const wada_vmc_msgs::msg::Can501_20221111::SharedPtr can501)
	{
		can501_ = *can501;
	}

	void callbackCurrPos(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr curr_pos)
	{
		mgrsToLatLon(curr_pos->pose.pose.position.y, curr_pos->pose.pose.position.x, latlon_.lat_, latlon_.lon_);

		tf2::Quaternion qua(curr_pos->pose.pose.orientation.x, curr_pos->pose.pose.orientation.y, curr_pos->pose.pose.orientation.z, curr_pos->pose.pose.orientation.w);
		tf2::Matrix3x3 matrix(qua);
		double roll, pitch, yaw;
		matrix.getRPY(roll, pitch, yaw);
		latlon_.car_yaw_ = yaw;

		tf2::Vector3 pose_plus(100.0, 0.0, 0.0);
		tf2::Quaternion qua_yaw;
		qua_yaw.setRPY(0.0, 0.0, -yaw);
		tf2::Transform tf_yaw(qua_yaw);
		tf2::Vector3 pose_plus_rot = tf_yaw * pose_plus;

		mgrsToLatLon(curr_pos->pose.pose.position.y - pose_plus_rot.getY(), curr_pos->pose.pose.position.x + pose_plus_rot.getX(), latlon_.lat1m_, latlon_.lon1m_);
	}

	void callbackCurrVel(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr curr_vel)
	{
		curr_vel_ = *curr_vel;
	}

	void callbackBlinker(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr blinker)
	{
		blinker_ = blinker->report;
	}

private://ros timer
	void callbackTimer()
	{
		//自動運転モード
		uint8_t pedal_mode = can501_.clutch;
		uint8_t steer_mode = can502_.clutch;

		//車両向き
		double car_yaw = azimuth(latlon_.lon_, latlon_.lat_, latlon_.lon1m_, latlon_.lat1m_) * M_PI / 180.0;

		//ウィンカー
		uint8_t mq_blinker = 0;
		if(blinker_ == autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT) mq_blinker = 1;
		else if(blinker_ == autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT) mq_blinker = 2;

		std::stringstream mqtt_message;
		mqtt_message << car_id_ << ',';//車両ID
		mqtt_message << date_.year << '-';//年
		mqtt_message << +date_.month << '-';//月
		mqtt_message << +date_.day << '_';//日
		mqtt_message << +date_.hour + 9 << '-';//時(JSTに変換)
		mqtt_message << +date_.min << '-';//分
		mqtt_message << std::fixed << std::setprecision(4) << date_.sec << ',';//秒
		mqtt_message << +pedal_mode << ',';//ペダルモード
		mqtt_message << +steer_mode << ',';//ステアモード
		mqtt_message << std::fixed << std::setprecision(7) << latlon_.lat_ << ',';//緯度
		mqtt_message << std::fixed << std::setprecision(7) << latlon_.lon_ << ',';//経度
		mqtt_message << std::fixed << std::setprecision(5) << car_yaw << ',';//車両向き
		mqtt_message << static_cast<int>(curr_vel_.twist.twist.linear.x * 3.6) << '.';//速度整数
		int  a = static_cast<int>(curr_vel_.twist.twist.linear.x * 3.6 * 10);
		int  b = static_cast<int>(std::floor(curr_vel_.twist.twist.linear.x * 3.6) * 10);
		mqtt_message << static_cast<int>(a-b) << ',';//速度少数
//		mqtt_message << static_cast<int>(can_velocity_param_.velocity * 3.6) << '.';//速度整数
//		mqtt_message << static_cast<int>(std::floor(can_velocity_param_.velocity * 3.6) * 10) << ',';//速度少数
		//mqtt_message << std::max(std::min(can503_.pedal_displacement, (int16_t)400), (int16_t)-280) << ',';//アクセル開度、ブレーキレベル
		//mqtt_message << static_cast<int>(can502_.angle_deg - 12.0) << ',';//ハンドル舵角
		//mqtt_message << std::max(std::min(can501_20221111_.pedal_pot_max, (int16_t)400), (int16_t)-280) << ',';//アクセル開度、ブレーキレベル
		mqtt_message << static_cast<int>(-((can501_.pedal_pot_average - 1028) / 1.55)) << ',';
		if(can502_.handle_pot_average >= 1024)
		{
			 mqtt_message << static_cast<int>((can502_.handle_pot_average - 1024) / 0.836363636) << ',';//ハンドル舵角
		}
		else
		{
			 mqtt_message << static_cast<int>((can502_.handle_pot_average - 1024) / 0.92) << ',';//ハンドル舵角
		}
		//mqtt_message << static_cast<int>(can502_20221111_.handle_angle) << ',';//ハンドル舵角
		mqtt_message << +mq_blinker << ',';//ウィンカー
		mqtt_message << "NULL" << ',';//シフト
		mqtt_message << "NULL" << ',';//バッテリー残量
		mqtt_message << "NULL" << ',';//パーキングブレーキ
		mqtt_message << "NULL" << ',';//車室内状態
		mqtt_message << "NULL" << ',';//ドア開閉状態
		mqtt_message << "00000000";//テルテール状態


		std::string mqtt_topic_name = "/vehicle/information";
		std::string mqtt_topic = mqtt_message.str();
		std::cout << mqtt_topic << std::endl;
		int ret = mosquitto_publish(mosq_, nullptr, mqtt_topic_name.c_str(), mqtt_topic.length(), mqtt_topic.c_str(), 0, false);
		if(ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_publish : error code " << ret << std::endl;
		}
		else
		{
			std::cout << "ok : mosquitto_publish topic name:" << mqtt_topic_name.c_str() << std::endl;
		}
	}

private://その他
	//Mosquittoの初期化
	void initMosquitto()
	{
		this->declare_parameter<std::string>("car_id", "0000");
		this->declare_parameter<std::string>("mqtt_host", "a1sjxnq4b8uinx-ats.iot.ap-northeast-1.amazonaws.com");
		this->declare_parameter<std::string>("tls_ca", "/home/sit/load_data/tokai_rika/AmazonRootCA1.pem");
		this->declare_parameter<std::string>("tls_crt", "/home/sit/load_data/tokai_rika/35568e97afba5f1cf52a78e2aadea54140909faf2a41096fbab66e29ca1cd411-certificate.pem.crt");
		this->declare_parameter<std::string>("tls_key", "/home/sit/load_data/tokai_rika/35568e97afba5f1cf52a78e2aadea54140909faf2a41096fbab66e29ca1cd411-private.pem.key");
		this->declare_parameter<std::string>("mgrs_utm_zone", "");
		this->declare_parameter<std::string>("mgrs_lat_band", "");
		this->declare_parameter<std::string>("mgrs_lattice_id", "");

		car_id_ = this->get_parameter("car_id").as_string();
		std::string mqtt_host = this->get_parameter("mqtt_host").as_string();
		std::string ca_file = this->get_parameter("tls_ca").as_string();
		std::string crt_file = this->get_parameter("tls_crt").as_string();
		std::string key_file = this->get_parameter("tls_key").as_string();
		mgrs_utm_zone_ = this->get_parameter("mgrs_utm_zone").as_string();
		mgrs_lat_band_ = this->get_parameter("mgrs_lat_band").as_string();
		mgrs_lattice_id_ = this->get_parameter("mgrs_lattice_id").as_string();

		RCLCPP_INFO(get_logger(), "car_id:%s", car_id_.c_str());
		RCLCPP_INFO(get_logger(), "mqtt_host:%s", mqtt_host.c_str());
		RCLCPP_INFO(get_logger(), "ca_file:%s", ca_file.c_str());
		RCLCPP_INFO(get_logger(), "crt_file:%s", crt_file.c_str());
		RCLCPP_INFO(get_logger(), "key_file:%s", key_file.c_str());
		RCLCPP_INFO(get_logger(), "mgrs_utm_zone:%s", mgrs_utm_zone_.c_str());
		RCLCPP_INFO(get_logger(), "mgrs_lat_band:%s", mgrs_lat_band_.c_str());
		RCLCPP_INFO(get_logger(), "mgrs_lattice_id:%s", mgrs_lattice_id_.c_str());

		//double lat, lon;
		//mgrsToLatLon(84032.1511, 250.6833, lat, lon);
		//RCLCPP_INFO(get_logger(), "lat,%lf  lon,%lf", lat, lon);

		//mqtt初期化
		int mqtt_ret = mosquitto_lib_init();
		if(mqtt_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_lib_init" << std::endl;
			return;
		}

		//mqttインスタンスの作成
		std::string mqtt_node_name = "mqtt_tokai_rika_pub";
		bool  clean_session = true;
		mosq_ = mosquitto_new(mqtt_node_name.c_str(), clean_session, nullptr);
		if(mosq_ == nullptr)
		{
			std::cout << "error : mosquitto_new" << std::endl;
			return;
		}

		//mqttコールバックのセット
		mosquitto_connect_callback_set(mosq_, TokaiRikaMqttPub::callback_on_connect);
		mosquitto_disconnect_callback_set(mosq_, TokaiRikaMqttPub::callback_on_disconnect);
		mosquitto_publish_callback_set(mosq_, TokaiRikaMqttPub::callback_on_publish);
		mosquitto_message_callback_set(mosq_, TokaiRikaMqttPub::callback_on_message);
		mosquitto_log_callback_set(mosq_, TokaiRikaMqttPub::callback_log);

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
		sub_date_ = this->create_subscription<orig_system_msgs::msg::Date>("topic_date", rclcpp::QoS(1),
			std::bind(&TokaiRikaMqttPub::callbackDate, this, std::placeholders::_1));
		sub_can501_ = this->create_subscription<wada_vmc_msgs::msg::Can501_20221111>("topic_can501", rclcpp::QoS(1),
			std::bind(&TokaiRikaMqttPub::callbackCan501, this, std::placeholders::_1));
		sub_can502_ = this->create_subscription<wada_vmc_msgs::msg::Can502_20221111>("topic_can502", rclcpp::QoS(1),
			std::bind(&TokaiRikaMqttPub::callbackCan502, this, std::placeholders::_1));
		sub_curr_pos_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("topic_curr_pos", rclcpp::QoS(1),
			std::bind(&TokaiRikaMqttPub::callbackCurrPos, this, std::placeholders::_1));
		sub_curr_vel_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("topic_curr_vel", rclcpp::QoS(1),
			std::bind(&TokaiRikaMqttPub::callbackCurrVel, this, std::placeholders::_1));
		sub_blinker_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>("topic_blinker", rclcpp::QoS(1),
			std::bind(&TokaiRikaMqttPub::callbackBlinker, this, std::placeholders::_1));

		timer_ = rclcpp::create_timer(
			this, get_clock(), rclcpp::Rate(2).period(), std::bind(&TokaiRikaMqttPub::callbackTimer, this));

		can501_.clutch = can502_.clutch = false;
	}

	//MGRS座標系を世界座標系に変換
	void mgrsToLatLon(const double x, const double y , double &lat, double &lon)
	{
		std::stringstream mgrs_code;
		double tmp;
		mgrs_code << mgrs_utm_zone_ << mgrs_lat_band_ << mgrs_lattice_id_;
		mgrs_code << std::setw(9) << std::setfill('0') << std::right << static_cast<int>(x * 1E4);
		mgrs_code << std::setw(9) << std::setfill('0') << std::right << static_cast<int>(y * 1E4);
		//RCLCPP_INFO(get_logger(), "code,%s", mgrs_code.str().c_str());
		int zone, prec;
		bool northp;
		double rx, ry;
		GeographicLib::MGRS::Reverse(mgrs_code.str(), zone, northp, rx, ry, prec);
		GeographicLib::UTMUPS::Reverse(zone, northp, rx, ry, lat, lon);
	}

	// 2点間の方位角を求める関数
	double azimuth(double x1, double y1, double x2, double y2)
	{
    	//Radian角に修正
		double _x1=x1*M_PI/180, _y1=y1*M_PI/180, _x2=x2*M_PI/180, _y2 =y2*M_PI/180;
		double delta_x = _x2 - _x1;
		double _y = std::sin(delta_x);
		double _x = std::cos(_y1) * std::tan(_y2) - std::sin(_y1) * std::cos(delta_x);
		double psi = std::atan2(_y, _x) * 180 / M_PI;
		if(psi < 0)
			return 360 + std::atan2(_y, _x) * 180 / M_PI;
		else
			return std::atan2(_y, _x) * 180 / M_PI;
	}

public:
	TokaiRikaMqttPub(const rclcpp::NodeOptions &node_options)
		: rclcpp::Node("tokairika_mqtt_pub", node_options)
		, init_flag_(false)
		, mosq_(nullptr)
	{
		initMosquitto();
		initRos2();
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	std::shared_ptr<TokaiRikaMqttPub> node = std::make_shared<TokaiRikaMqttPub>(options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
