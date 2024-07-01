#include <rclcpp/rclcpp.hpp>
#include <wada_vmc_msgs/msg/can502_20210103.hpp>
#include <wada_vmc_msgs/msg/steer_cmd.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

namespace car_auto_operation
{
	class CarAutoOperationSteer20210103 : public rclcpp::Node
	{
	private://ros2 subscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::Can502_20210103>::SharedPtr sub_can502_;//ステア情報
		rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_control_;//AUTOWAREからのコマンド

	private://ros2 publisher
		rclcpp::Publisher<wada_vmc_msgs::msg::SteerCmd>::SharedPtr pub_steer_cmd_;//canに送信する

	private://ros2 timer
		rclcpp::TimerBase::SharedPtr timer_;

	private://車両固有情報
		double wheelrad_to_steering_can_value_left_;
		double wheelrad_to_steering_can_value_right_;
		double steer_ang_input_center_;//joyボードのステア角度中央値

	private://autowareの変数
		std::vector<wada_vmc_msgs::msg::Can502_20210103> can502_;//ステア情報
		std::vector<rclcpp::Time> can502_time_;//can502のsubscribeした時間

	private://ros2 callback
		void callbackCan502(const wada_vmc_msgs::msg::Can502_20210103::SharedPtr can)
		{
			rclcpp::Time rosnowtime = this->now();
			can502_.insert(can502_.begin(), *can);
			can502_time_.insert(can502_time_.begin(), rosnowtime);
		}

		void callbackCommand(autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr command)
		{
			wada_vmc_msgs::msg::SteerCmd::UniquePtr steer_cmd = std::make_unique<wada_vmc_msgs::msg::SteerCmd>();
			steer_cmd->stamp = this->now();

			const float cmd_tire_angle = command->lateral.steering_tire_angle;
			if(cmd_tire_angle > 0)
			{
				steer_cmd->steer_cmd = cmd_tire_angle * wheelrad_to_steering_can_value_left_;
				RCLCPP_INFO(this->get_logger(), "left %d,%lf",steer_cmd->steer_cmd, wheelrad_to_steering_can_value_left_);
			}
			else
			{
				steer_cmd->steer_cmd = cmd_tire_angle * wheelrad_to_steering_can_value_left_;
				RCLCPP_INFO(this->get_logger(), "right %d,%lf",steer_cmd->steer_cmd, wheelrad_to_steering_can_value_right_);
			}
			//steer_cmd->steer_cmd = std::min(std::max(steer_cmd->steer_cmd, steer_input_right_min_), steer_input_left_max_);

			pub_steer_cmd_->publish(std::move(steer_cmd));
		}

	private://ros2 timer callback
		void callbackTimer()
		{
			if(can502_.size() == 0) return;

			rclcpp::Time nowtime = this->now();
			for(size_t i=0; i<can502_.size(); i++)
			{
				if(nowtime - can502_time_[i] > rclcpp::Duration(1, 0))
				{
					can502_.erase(can502_.begin() + i, can502_.end());
					can502_time_.erase(can502_time_.begin() + i, can502_time_.end());
					break;
				}
			}
		}

	public:
		CarAutoOperationSteer20210103(const rclcpp::NodeOptions options)
			: rclcpp::Node("car_auto_operation_steer_20210103", options)
		{
			this->declare_parameter<double>("wheelrad_to_steering_can_value_left", 25009.6727514125);
			this->declare_parameter<double>("wheelrad_to_steering_can_value_right", 26765.9140133745);
			this->declare_parameter<double>("steer_ang_input_center", 0.0);

			wheelrad_to_steering_can_value_left_ = get_parameter("wheelrad_to_steering_can_value_left").as_double();
			wheelrad_to_steering_can_value_right_ = get_parameter("wheelrad_to_steering_can_value_right").as_double();
			steer_ang_input_center_ = get_parameter("steer_ang_input_center").as_double();

			RCLCPP_INFO(this->get_logger(), "wheelrad_to_steering_can_value_left,%lf", wheelrad_to_steering_can_value_left_);
			RCLCPP_INFO(this->get_logger(), "wheelrad_to_steering_can_value_left,%lf", wheelrad_to_steering_can_value_right_);
			RCLCPP_INFO(this->get_logger(), "steer_ang_input_center,%lf", steer_ang_input_center_);

			sub_can502_ = this->create_subscription<wada_vmc_msgs::msg::Can502_20210103>("can502", rclcpp::QoS(1),
				std::bind(&CarAutoOperationSteer20210103::callbackCan502, this, std::placeholders::_1));
			sub_control_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(1),
				std::bind(&CarAutoOperationSteer20210103::callbackCommand, this, std::placeholders::_1));

			pub_steer_cmd_ = this->create_publisher<wada_vmc_msgs::msg::SteerCmd>("steer_cmd",rclcpp::QoS(1));

			timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Rate(30).period(), std::bind(&CarAutoOperationSteer20210103::callbackTimer, this));
		}
	};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(car_auto_operation::CarAutoOperationSteer20210103)