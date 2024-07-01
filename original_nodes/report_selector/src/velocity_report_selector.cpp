#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

class VelocityReportSelector : public rclcpp::Node
{
private://ros2 publisher
	rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_vel_report;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_select_report_;

private://ros2 subscriber
	rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_vel_can_;
	rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_vel_gnss_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_solution_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_backlamp_;

private://メンバ変数
	std::vector<autoware_auto_vehicle_msgs::msg::VelocityReport> vel_can_;
	std::vector<autoware_auto_vehicle_msgs::msg::VelocityReport> vel_gnss_;
	std::vector<double> vel_gnss_acc_;
	std::vector<rclcpp::Time> vel_gnss_time_;
	double vel_gnss_acc_avg_;
	std::string solution_;
	const double can_vel_th_kmh_;
	bool backlamp_;

private://ros2 callback
	void callbackVelCan(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr report)
	{
		vel_can_.push_back(*report);

		if(vel_can_.size() > 10)
		{
			vel_can_.erase(vel_can_.begin());
		}

		selectReport();
	}

	void callbackVelGnss(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr report)
	{
		rclcpp::Time nowtime = this->now();

		if(vel_gnss_.size() >= 1)
			vel_gnss_acc_.push_back((report->longitudinal_velocity - vel_gnss_[vel_gnss_.size() - 1].lateral_velocity) / (nowtime.seconds() - vel_gnss_time_[vel_gnss_time_.size()-1].seconds()));

		vel_gnss_.push_back(*report);
		vel_gnss_time_.push_back(nowtime);

		if(vel_gnss_.size() > 10)
		{
			vel_gnss_.erase(vel_gnss_.begin());
			vel_gnss_time_.erase(vel_gnss_time_.begin());
			vel_gnss_acc_.erase(vel_gnss_acc_.begin());
		}

		vel_gnss_acc_avg_ = 0;
		for(double acc : vel_gnss_acc_)
			vel_gnss_acc_avg_ += acc;
		if(vel_gnss_acc_.size() > 0) vel_gnss_acc_avg_ /= vel_gnss_acc_.size();
	}

	void callbackSolution(const std_msgs::msg::String::ConstSharedPtr sol)
	{
		solution_ = sol->data;
	}

	void callbackBackLamp(const std_msgs::msg::Bool::ConstSharedPtr lamp)
	{
		backlamp_ = lamp->data;
	}

private://その他
	void selectReport()
	{
		std_msgs::msg::String msg_select;
		if(vel_can_.size() == 0 && vel_gnss_.size() == 0)
		{
			msg_select.data = "NONE";
			pub_select_report_->publish(msg_select);
			return;
		}
		else if(vel_can_.size() != 0 && vel_gnss_.size() == 0)
		{
			pub_vel_report->publish(vel_can_[vel_can_.size()-1]);
			msg_select.data = "CAN(NO GNSS)";
			pub_select_report_->publish(msg_select);
			return;
		}

		const double v_th = can_vel_th_kmh_ / 3.6;
		const double can_vel = vel_can_[vel_can_.size()-1].longitudinal_velocity;
		const double gnss_vel = vel_gnss_[vel_gnss_.size()-1].longitudinal_velocity;
		autoware_auto_vehicle_msgs::msg::VelocityReport pub_vel;
		std::stringstream ss_select;
		if(can_vel < v_th && can_vel > gnss_vel && vel_gnss_acc_avg_ < 0)
		{
			pub_vel = vel_gnss_[vel_gnss_.size()-1];
			ss_select << "GNSS(SPEED DOWN)," << pub_vel.longitudinal_velocity << ',' << can_vel << ',' << gnss_vel;
		}
		else if(can_vel < v_th && can_vel < gnss_vel && vel_gnss_acc_avg_ > 0)
		{
			pub_vel = vel_gnss_[vel_gnss_.size()-1];
			ss_select << "GNSS(SPEED UP)," << pub_vel.longitudinal_velocity << ',' << can_vel << ',' << gnss_vel;
		}
		else
		{
			pub_vel = vel_can_[vel_can_.size()-1];
			if(backlamp_ == true) pub_vel.longitudinal_velocity *= -1;
			ss_select << "CAN(NORMAL)," << pub_vel.longitudinal_velocity << ',' << can_vel << ',' << gnss_vel;
		}
		pub_vel_report->publish(pub_vel);
		msg_select.data = ss_select.str();
		pub_select_report_->publish(msg_select);
		/*std_msgs::msg::String msg_select;
		if(vel_can_.size() == 0 && vel_gnss_.size() == 0)
		{
			msg_select.data = "NONW";
			pub_select_report_->publish(msg_select);
			return;
		}
		else if(vel_can_.size() != 0 && vel_gnss_.size() == 0)
		{
			pub_vel_report->publish(vel_can_[vel_can_.size()-1]);
			msg_select.data = "CAN(NO GNSS)";
			pub_select_report_->publish(msg_select);
		}
		else
		{
			float can_v = vel_can_[vel_can_.size()-1].longitudinal_velocity;
			float gnss_v = vel_gnss_[vel_gnss_.size()-1].longitudinal_velocity;

			autoware_auto_vehicle_msgs::msg::VelocityReport msg_report;
			msg_select.data = "CAN(NORMAL)";

			msg_report = vel_can_[vel_can_.size()-1];
			if(can_v < can_vel_th_kmh_ / 3.6)
			{
				if(can_v > gnss_v)
				{
					if(vel_can_acc_ < 0.0)
					{
						if(solution_ == "INS_SOLUTION_GOOD")
						{
							vel_gnss_[vel_gnss_.size()-1];
							msg_select.data = "GNSS(SPEED DOWN)";
						}
					}
				}
				else
				{
					if(vel_can_acc_ > 0.0)
					{
						if(solution_ == "INS_SOLUTION_GOOD")
						{
							vel_gnss_[vel_gnss_.size()-1];
							msg_select.data = "GNSS(SPEED UP)";
						}
					}
				}
			}

			pub_vel_report->publish(msg_report);
			pub_select_report_->publish(msg_select);
		}*/

		//can_v < th && can_v > gnss_v && acc_v < 0 && sol < good;
		//can_v < th && can_v < gnss_v && acc_v > 0 && sol < good;
	}

public:
	VelocityReportSelector(const rclcpp::NodeOptions &node_options)
		: rclcpp::Node("velocity_report_selector", node_options)
		, vel_gnss_acc_(0.0)
		, can_vel_th_kmh_(this->declare_parameter<double>("can_vel_th_kmh", 7.0))
		, backlamp_(false)
	{
		pub_vel_report = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("out_vel", rclcpp::QoS(1));
		pub_select_report_ = create_publisher<std_msgs::msg::String>("select_report", rclcpp::QoS(1));

		sub_vel_can_ = create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("in_vel_can", rclcpp::QoS(1),
			std::bind(&VelocityReportSelector::callbackVelCan, this, std::placeholders::_1));
		sub_vel_gnss_ = create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>("in_vel_gnss", rclcpp::QoS(1),
			std::bind(&VelocityReportSelector::callbackVelGnss, this, std::placeholders::_1));
		sub_solution_ = create_subscription<std_msgs::msg::String>("in_gnss_solution", rclcpp::QoS(1),
			std::bind(&VelocityReportSelector::callbackSolution, this, std::placeholders::_1));
		sub_backlamp_ = create_subscription<std_msgs::msg::Bool>("backlamp", rclcpp::QoS(1),
			std::bind(&VelocityReportSelector::callbackBackLamp, this, std::placeholders::_1));
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<VelocityReportSelector> node = std::make_shared<VelocityReportSelector>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}