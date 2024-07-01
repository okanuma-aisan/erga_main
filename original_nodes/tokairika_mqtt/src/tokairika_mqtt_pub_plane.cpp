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
	std::string car_id_ = "0001";//mqttで送信する車両ID
	orig_system_msgs::msg::Date date_;//GNSSから送信される時間(/gnss_time)
	wada_vmc_msgs::msg::Can501_20221111 can501_;//joy boardのペダル情報
	wada_vmc_msgs::msg::Can502_20221111 can502_;//joy boardのハンドル情報
	LatLon latlon_;//緯度経度
	geometry_msgs::msg::TwistWithCovarianceStamped curr_vel_;//現在車両速度
	uint8_t blinker_;//ウィンカー情報

private://その他の変数
	uint8_t plane_number_;//直行座標系の番号

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
		planeToLatlon(curr_pos->pose.pose.position.y, curr_pos->pose.pose.position.x, plane_number_, latlon_.lat_, latlon_.lon_);//autowareの直交座標系はXYが通常と逆

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
		planeToLatlon(curr_pos->pose.pose.position.y - pose_plus_rot.getY(), curr_pos->pose.pose.position.x + pose_plus_rot.getX(),
			plane_number_, latlon_.lat1m_, latlon_.lon1m_);

		RCLCPP_INFO(this->get_logger(), "yaw,%lf", yaw);
		RCLCPP_INFO(this->get_logger(), "lat  ,%lf  lon  ,%lf", latlon_.lat_, latlon_.lon_);
		RCLCPP_INFO(this->get_logger(), "lat1m,%lf  lon1m,%lf", latlon_.lat1m_, latlon_.lon1m_);
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
		this->declare_parameter("mqtt_host", "a1sjxnq4b8uinx-ats.iot.ap-northeast-1.amazonaws.com");
		this->declare_parameter("tls_ca", "/home/sit/load_data/tokai_rika/AmazonRootCA1.pem");
		this->declare_parameter("tls_crt", "/home/sit/load_data/tokai_rika/35568e97afba5f1cf52a78e2aadea54140909faf2a41096fbab66e29ca1cd411-certificate.pem.crt");
		this->declare_parameter("tls_key", "/home/sit/load_data/tokai_rika/35568e97afba5f1cf52a78e2aadea54140909faf2a41096fbab66e29ca1cd411-private.pem.key");
		this->declare_parameter("plane_number", 7);
		//this->declare_parameter("car_id", "0001");

		std::string mqtt_host = this->get_parameter("mqtt_host").as_string();
		std::string ca_file = this->get_parameter("tls_ca").as_string();
		std::string crt_file = this->get_parameter("tls_crt").as_string();
		std::string key_file = this->get_parameter("tls_key").as_string();
		plane_number_ = static_cast<uint8_t>(this->get_parameter("plane_number").as_int());
		//car_id_ = this->get_parameter("car_id").as_string();

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

	//直交座標系を緯度経度座標系に変換
	void planeToLatlon(const double x, const double y, const uint8_t plane, double &ido, double &keido)
	{
		const double a=6378137, rf=298.257222101, m0=0.9999, s2r=M_PI/648000, n=0.5/(rf-0.5);
		const double n15=1.5*n, anh=0.5*a/(1+n), nsq=n*n, ra=2*anh*m0*(1+nsq/4+nsq*nsq/64);
		const int jt=5, jt2=2*jt;
		double ep=1.0;
		double e[jt2+1], s[jt2+1+1], t[jt2+1], beta[5+1], dlt[6+1];
		s[0] = 0;

		for(int k=1; k<=jt; k++)
		{
			e[k] = n15/k-n;
			ep *= e[k];
			e[k+jt] = n15/(k+jt)-n;
		}

		// 展開パラメータの事前入力
		beta[1]=(1.0/2.0+(-2.0/3.0+(37.0/96.0+(-1.0/360.0-81.0/512.0*n)*n)*n)*n)*n;
		beta[2]=(1.0/48.0+(1.0/15.0+(-437.0/1440.0+46.0/105.0*n)*n)*n)*nsq;
		beta[3]=(17.0/480.0+(-37.0/840.0-209.0/4480.0*n)*n)*n*nsq;
		beta[4]=(4397.0/161280.0-11.0/504.0*n)*nsq*nsq;
		beta[5]=4583.0/161280.0*n*nsq*nsq;
		dlt[1]=(2.0+(-2.0/3.0+(-2.0+(116.0/45.0+(26.0/45.0-2854.0/675.0*n)*n)*n)*n)*n)*n;
		dlt[2]=(7.0/3.0+(-8.0/5.0+(-227.0/45.0+(2704.0/315.0+2323.0/945.0*n)*n)*n)*n)*nsq;
		dlt[3]=(56.0/15.0+(-136.0/35.0+(-1262.0/105.0+73814.0/2835.0*n)*n)*n)*n*nsq;
		dlt[4]=(4279.0/630.0+(-332.0/35.0-399572.0/14175.0*n)*n)*nsq*nsq;
		dlt[5]=(4174.0/315.0-144838.0/6237.0*n)*n*nsq*nsq;
		dlt[6]=601676.0/22275.0*nsq*nsq*nsq;

		// 平面直角座標の座標系原点の緯度を度単位で、経度を分単位で格納
		double phi0[] = { 0,33,33,36,33,36,36,36,36,36,40,44,44,44,26,26,26,26,20,26};
		double lmbd0[] = {0,7770,7860,7930,8010,8060,8160,8230,8310,8390,8450,8415,8535,8655,8520,7650,7440,7860,8160,9240};

		// 該当緯度の 2 倍角の入力により赤道からの子午線弧長を求める関数
		auto Merid = [](const double phi2, const double ep, const double anh, const double *e, double *s, double *t)
		{
			double dc=2.0*std::cos(phi2);
			s[1]=std::sin(phi2);
			for(int i=1; i<=jt2; i++)
			{
				s[i+1]=dc*s[i]-s[i-1];
				t[i]=(1.0/i-4.0*i)*s[i];
			}
			double sum=0.0, c1=ep;
			int j=jt;
			while(j > 0) {
				double c2=phi2, c3=2.0;
				int l=j, m=0;
				while(l > 0)
				{
					c2+=(c3/=e[l--])*t[++m]+(c3*=e[2*j-l])*t[++m];
#if 0					
					c3/=e[l];
					m++;
					c3*=e[2*j-l];
					c2+=(c3)*t[m];
					l--;
					m++;
					c2+=(c3)*t[m];
#endif					
				}
				sum+=c1*c1*c2 ; c1/=e[j];
				j--;
			}
			// << anh << "," << sum << "," << phi2 << "," << ep << std::endl;
			return anh*(sum+phi2);
		};

		//x = 11573.375;
		//y = 22694.380;
		// 実際の計算実行部分
		double xi = (x+m0*Merid(2.0*phi0[plane]*3600.0*s2r,ep,anh,e,s,t))/ra, eta=y/ra, sgmp=1, taup=0;
		//std::cout << "xi," << xi << std::endl;
		double xip = xi, etap = eta;
		for(int j=5; j>0; --j ) {
			double besin=beta[j]*std::sin(2*j*xi), becos=beta[j]*std::cos(2*j*xi);
			xip-=besin*std::cosh(2*j*eta); etap-=becos*std::sinh(2*j*eta);
			sgmp-=2*j*becos*std::cosh(2*j*eta); taup+=2*j*besin*std::sinh(2*j*eta);
		}

		double sxip=std::sin(xip), cxip=std::cos(xip), shetap=std::sinh(etap), chetap=std::cosh(etap);
		double chi=std::asin(sxip/chetap);
		double phi = chi;
		for(int j=6; j>0; --j ) { phi+=dlt[j]*std::sin(2*j*chi); }
		double nphi=(1-n)/(1+n)*std::tan(phi);

		double lmbd=lmbd0[plane]*60.0+std::atan2(shetap, cxip)/s2r;
		double gmm=std::atan2(taup*cxip*chetap+sgmp*sxip*shetap,sgmp*cxip*chetap-taup*sxip*shetap);
		double m=ra/a*std::sqrt((cxip*cxip+shetap*shetap)/(sgmp*sgmp+taup*taup)*(1+nphi*nphi));

		// ラジアン → 度分秒変換
		/*iza=std::floo(phi/s2r/3600);
		ifun =std::floor((phi/s2r-iza*3600)/60);
		ibyou =(phi/s2r-iza*3600-ifun*60)/60.0;
		keiza =std::floor(lmbd/3600);
		keifun =std::floor((lmbd-keiza*3600)/60);
		keibyou =(lmbd-keiza*3600-keifun*60)/60.0;*/
		ido = phi/s2r/3600;
		keido = lmbd/3600;
		double sgn=(gmm<0);
		double gdo=std::floor(gmm/s2r/3600)+sgn;
		double gfun=std::floor((gmm/s2r-gdo*3600)/60)+sgn;
		double gbyou=gmm/s2r-gdo*3600-gfun*60;

//		std::cout << "phi," << phi/s2r/3600 << "  lmd," << keido << std::endl;
		//std::cout << "phi=" << iza << "°" << ifun << "'" << ibyou << "\"  lmd=" << keiza << "°" << keifun << "'" << keibyou << "\"" << std::endl;
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
