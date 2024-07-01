#include <rclcpp/rclcpp.hpp>
#include <report_selector_msgs/msg/report_select.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <nmea_msgs/msg/gpgga.hpp>

class ReportSelector : public rclcpp::Node
{
private://定数
	/*constexpr static uint8_t VRS_NONE = 0;//速度レポートをpublishしない
	constexpr static uint8_t VRS_MAIN = 1;//sub_velocity_report_main_でsubscribeしている速度レポートをautowareに渡す
	constexpr static uint8_t VRS_SUB = 2; //sub_velocity_report_sub_でsubscribeしている速度レポートをautowareに渡す

	constexpr static uint8_t SRS_NONE = 0;//タイヤ角レポートをpublishしない
	constexpr static uint8_t SRS_MAIN = 1;//sub_steering_report_main_でsubscribeしているタイヤ角レポートをautowareに渡す
	constexpr static uint8_t SRS_SUB = 2; //sub_steering_report_sub_でsubscribeしているタイヤ角レポートをautowareに渡す
	*/

	constexpr static double REPORT_CHECK_TIMER_HZ = 20;//report_check_timer_の周期

private://ros2 subscriber
	rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_report_main_;//Mainのvelocityレポート 通常時はこちらをautowareの速度レポートとしてpublishする
	rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_report_sub_;//Subのvelocityレポート Mainに異常が生じた場合は、こちらをautowareの速度レポートとしてpublishする
	rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steering_report_main_;//Mainのsteeringレポート 通常時はこちらをautowareの速度レポートとしてpublishする
	rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steering_report_sub_;//Subのsteeringレポート Mainに異常が生じた場合は、こちらをautowareの速度レポートとしてpublishする
	rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gpgga_main_;//hdopを取得するgpggaセンテンス(Main)
	rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gpgga_sub_;//hdopを取得するgpggaセンテンス(Main)

private://ros2 publisher
	rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_velocity_report_;//autowareに渡すvelocityレポート
	rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steering_report_;//autowareに渡すvelocityレポート
	rclcpp::Publisher<report_selector_msgs::msg::ReportSelect>::SharedPtr pub_report_select_;//レポート選択状態をpublish 死活監視用

private://ros2 timer
	rclcpp::TimerBase::SharedPtr report_check_timer_;//subscribeしたレポートの状態をチェックして、使用するレポートの選択を行うタイマー

private://report選択用メンバ変数
	bool use_gpgga_main_;//Main側でgpggaトピックを使用するか？
	bool use_gpgga_sub_; //Sub側でgpggaトピックを使用するか
	float hdop_th_main_;//gpggaのhdopがこの数値を超えたら場合はレポートを扱わない(Main)
	float hdop_th_sub_;//gpggaのhdopがこの数値を超えたら場合はレポートを扱わない(Sub)
	double diff_time_velocity_th_main_;//Mainのvelocityレポートのsubscribe時間が現在時刻とこの秒数まで離れていたら、sub側のvelocityレポートを使用する
	double diff_time_velocity_th_sub_;//Subのvelocityレポートのsubscribe時間が現在時刻とこの秒数まで離れていたら、エラーにする
	double diff_time_steering_th_main_;//Mainのsteeringレポートのsubscribe時間が現在時刻とこの秒数まで離れていたら、sub側のsteeringレポートを使用する
	double diff_time_steering_th_sub_;//Subのsteeringレポートのsubscribe時間が現在時刻とこの秒数まで離れていたら、エラーにする

	//uint8_t velocity_report_select_;//mainとsubの速度レポートのどちらをautowareに渡すかのフラグ(VRS定数)
	//uint8_t steering_report_select_;//mainとsubのタイヤ角レポートのどちらをautowareに渡すかのフラグ(VRS定数)
	report_selector_msgs::msg::ReportSelect report_select_;//選択されているレポート状態(MainかSubか)
	rclcpp::Time velocity_main_time_;//sub_velocity_report_main_のsubscribe時間
	rclcpp::Time velocity_sub_time_;//sub_velocity_report_sub_のsubscribe時間
	rclcpp::Time steering_main_time_;//sub_steering_report_main_のsubscribe時間
	rclcpp::Time steering_sub_time_;//sub_steering_report_sub_のsubscribe時間

	float hdop_main_;//gpggaセンテンスから取得するHDOP値(Main)
	float hdop_sub_;//gpggaセンテンスから取得するHDOP値(Sub)

private://ros2 callback
	//Mainのvelocityレポート 通常時はこちらをautowareの速度レポートとしてpublishする
	void callbackVelocityReportMain(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr report)
	{
		if(report_select_.velocity_report_select == report_selector_msgs::msg::ReportSelect::VRS_MAIN)
			pub_velocity_report_->publish(*report);
		velocity_main_time_ = report->header.stamp;
	}

	//Subのvelocityレポート Mainに異常が生じた場合は、こちらをautowareの速度レポートとしてpublishする
	void callbackVelocityReportSub(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr report)
	{
		if(report_select_.velocity_report_select == report_selector_msgs::msg::ReportSelect::VRS_SUB)
			pub_velocity_report_->publish(*report);
		velocity_sub_time_ = report->header.stamp;
	}

	//Mainのsteeringレポート 通常時はこちらをautowareのタイヤ角レポートとしてpublishする
	void callbackSteeringReportMain(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr report)
	{
		if(report_select_.steering_report_select == report_selector_msgs::msg::ReportSelect::SRS_MAIN)
			pub_steering_report_->publish(*report);
		steering_main_time_ = report->stamp;
	}

	//Subのsteeringレポート Mainに異常が生じた場合は、こちらをautowareの速度レポートとしてpublishする
	void callbackSteeringReportSub(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr report)
	{
		if(report_select_.steering_report_select == report_selector_msgs::msg::ReportSelect::SRS_SUB)
			pub_steering_report_->publish(*report);
		steering_sub_time_ = report->stamp;
	}

	//GPGGAセンテンス
	void callbackGpggaMain(const nmea_msgs::msg::Gpgga::ConstSharedPtr gpgga)
	{
		hdop_main_ = gpgga->hdop;
	}
	void callbackGpggaSub(const nmea_msgs::msg::Gpgga::ConstSharedPtr gpgga)
	{
		hdop_sub_ = gpgga->hdop;
	}

	void callbackReportCheckTimer()
	{
		double nowtime = this->now().seconds();

		{
			bool hdop_main_flag = true;
			if(use_gpgga_main_)
				if(hdop_main_ > hdop_th_main_) hdop_main_flag = false;

			bool velocity_main_time_flag = true;
			if(nowtime - velocity_main_time_.seconds() > diff_time_velocity_th_main_)
				velocity_main_time_flag = false;

			if(hdop_main_flag && velocity_main_time_flag)
				report_select_.velocity_report_select = report_selector_msgs::msg::ReportSelect::VRS_MAIN;
			else
			{
				bool hdop_sub_flag = true;
				if(use_gpgga_sub_)
					if(hdop_sub_ > hdop_th_sub_) hdop_sub_flag = false;

				bool velocity_sub_time_flag = true;
				if(nowtime - velocity_sub_time_.seconds() > diff_time_velocity_th_sub_)
					velocity_sub_time_flag = false;

				if(hdop_sub_flag && velocity_sub_time_flag)
					report_select_.velocity_report_select = report_selector_msgs::msg::ReportSelect::VRS_SUB;
				else
					report_select_.velocity_report_select = report_selector_msgs::msg::ReportSelect::VRS_NONE;
			}
		}

		{
			bool steering_main_time_flag = true;
			if(nowtime - steering_main_time_.seconds() > diff_time_steering_th_main_)
				steering_main_time_flag = false;

			if(steering_main_time_flag)
				report_select_.steering_report_select = report_selector_msgs::msg::ReportSelect::SRS_MAIN;
			else
			{
				bool steering_sub_time_flag = true;
				if(nowtime - steering_sub_time_.seconds() > diff_time_steering_th_sub_)
					steering_sub_time_flag = false;

				if(steering_sub_time_flag)
					report_select_.steering_report_select = report_selector_msgs::msg::ReportSelect::SRS_SUB;
				else
					report_select_.steering_report_select = report_selector_msgs::msg::ReportSelect::SRS_NONE;
			}
		}

		pub_report_select_->publish(report_select_);

		/*if(nowtime - velocity_main_time_.seconds() > diff_time_velocity_th_main_)
		{
			if(nowtime - velocity_sub_time_.seconds() > diff_time_velocity_th_sub_)
				report_select_.velocity_report_select = VRS_NONE;
			else
				report_select_.velocity_report_select = VRS_SUB;
		}
		else report_select_.velocity_report_select = VRS_MAIN;

		if(nowtime - steering_main_time_.seconds() > diff_time_steering_th_main_)
		{
			if(nowtime - velocity_sub_time_.seconds() > diff_time_steering_th_sub_)
				report_select_.steering_report_select = SRS_NONE;
			else
				report_select_.steering_report_select = SRS_SUB;
		}
		else report_select_.steering_report_select = SRS_MAIN;

		pub_report_select_->publish(report_select_);*/
	}

public:
	ReportSelector(const rclcpp::NodeOptions &node_optiopns)
		: rclcpp::Node("report_selector", node_optiopns)
		, velocity_main_time_(rclcpp::Time(0))
		, velocity_sub_time_(rclcpp::Time(0))
		, steering_main_time_(rclcpp::Time(0))
		, steering_sub_time_(rclcpp::Time(0))
		, hdop_main_(100.0)
		, hdop_sub_(100.0)
	{
		declare_parameter<bool>("use_gpgga_main", false);
		declare_parameter<bool>("use_gpgga_sub", false);
		declare_parameter<double>("hdop_th_main", 1.0);
		declare_parameter<double>("hdop_th_sub", 1.0);
		declare_parameter<double>("diff_time_velocity_th_main", 0.5);
		declare_parameter<double>("diff_time_velocity_th_sub", 0.5);
		declare_parameter<double>("diff_time_steering_th_main", 0.5);
		declare_parameter<double>("diff_time_steering_th_sub", 0.5);
		use_gpgga_main_ = get_parameter("use_gpgga_main").as_bool();
		use_gpgga_sub_ = get_parameter("use_gpgga_sub").as_bool();
		hdop_th_main_ = static_cast<float>(get_parameter("hdop_th_main").as_double());
		hdop_th_sub_ = static_cast<float>(get_parameter("hdop_th_sub").as_double());
		diff_time_velocity_th_main_ = get_parameter("diff_time_velocity_th_main").as_double();
		diff_time_velocity_th_sub_ = get_parameter("diff_time_velocity_th_sub").as_double();
		diff_time_steering_th_main_ = get_parameter("diff_time_steering_th_main").as_double();
		diff_time_steering_th_sub_ = get_parameter("diff_time_steering_th_sub").as_double();
		RCLCPP_INFO(this->get_logger(), "use_gpgga_main:%s", (use_gpgga_main_ ? "true" : "false"));
		RCLCPP_INFO(this->get_logger(), "use_gpgga_sub:%s", (use_gpgga_sub_ ? "true" : "false"));
		RCLCPP_INFO(this->get_logger(), "hdop_th_main:%lf", hdop_th_main_);
		RCLCPP_INFO(this->get_logger(), "hdop_th_sub:%lf", hdop_th_sub_);
		RCLCPP_INFO(this->get_logger(), "diff_time_velocity_th_main:%lf", diff_time_velocity_th_main_);
		RCLCPP_INFO(this->get_logger(), "diff_time_velocity_th_sub:%lf", diff_time_velocity_th_sub_);
		RCLCPP_INFO(this->get_logger(), "diff_time_steering_th_main:%lf", diff_time_steering_th_main_);
		RCLCPP_INFO(this->get_logger(), "diff_time_steering_th_sub:%lf", diff_time_steering_th_sub_);

		sub_velocity_report_main_ = create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("input_velocity_report_main",
			rclcpp::QoS(1), std::bind(&ReportSelector::callbackVelocityReportMain, this, std::placeholders::_1));
		sub_velocity_report_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("input_velocity_report_sub",
			rclcpp::QoS(1), std::bind(&ReportSelector::callbackVelocityReportSub, this, std::placeholders::_1));
		sub_steering_report_main_ = create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>("input_steering_report_main",
			rclcpp::QoS(1), std::bind(&ReportSelector::callbackSteeringReportMain, this, std::placeholders::_1));
		sub_steering_report_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>("input_steering_report_sub",
			rclcpp::QoS(1), std::bind(&ReportSelector::callbackSteeringReportSub, this, std::placeholders::_1));

		if(use_gpgga_main_)
		{
			sub_gpgga_main_ = create_subscription<nmea_msgs::msg::Gpgga>("gpgga_main",
				rclcpp::QoS(1), std::bind(&ReportSelector::callbackGpggaMain, this, std::placeholders::_1));
		}
		if(use_gpgga_sub_)
		{
			sub_gpgga_sub_ = create_subscription<nmea_msgs::msg::Gpgga>("gpgga_sub",
				rclcpp::QoS(1), std::bind(&ReportSelector::callbackGpggaSub, this, std::placeholders::_1));
		}

		pub_velocity_report_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("output_velocity_report", rclcpp::QoS(1));
		pub_steering_report_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("output_steering_report", rclcpp::QoS(1));
		pub_report_select_ = create_publisher<report_selector_msgs::msg::ReportSelect>("report_select", rclcpp::QoS(1));

		report_check_timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(REPORT_CHECK_TIMER_HZ).period(),
				std::bind(&ReportSelector::callbackReportCheckTimer, this));
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<ReportSelector> node = std::make_shared<ReportSelector>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}