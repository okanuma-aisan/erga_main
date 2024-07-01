#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <wada_vmc_msgs/msg/can502_20221111.hpp>
#include <wada_vmc_msgs/msg/steer_cmd.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

namespace car_auto_operation
{
	class CarAutoOperationSteer : public rclcpp::Node
	{
	private://ros2 subscriber
		rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_control_;//AUTOWAREからのコマンド
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_beta_;//横滑り角

	private://ros2 publisher
		rclcpp::Publisher<wada_vmc_msgs::msg::SteerCmd>::SharedPtr pub_steer_cmd_;//canに送信する

	private://ros2 timer
		rclcpp::TimerBase::SharedPtr timer_;

	private://車両固有情報
		double tire_rad_to_steering_can_value_left_slope_;//タイヤ角からステアのCAN指令に変換する係数(左傾き)
		double tire_rad_to_steering_can_value_right_slope_;//タイヤ角からステアのCAN指令に変換する係数(右傾き)
		double tire_rad_to_steering_can_value_left_intercept_;//タイヤ角からステアのCAN指令に変換する係数(左切片)
		double tire_rad_to_steering_can_value_right_intercept_;//タイヤ角からステアのCAN指令に変換する係数(右切片)
		int16_t steer_input_left_max_;//ステアCAN指令の最大値(左周り)
		int16_t steer_input_right_min_;//ステアCAN指令の最小値(右周り)
		int16_t steer_input_center_;//ステア中立値
		int16_t steer_input_correction_send_;//ステア補正値(送信側)

	private:
		bool use_beta_;//横滑り角を使用するか？
		double beta_;//横滑り角

	private://ros2 callback
		void callbackCommand(autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr command)
		{
			wada_vmc_msgs::msg::SteerCmd::UniquePtr steer_cmd = std::make_unique<wada_vmc_msgs::msg::SteerCmd>();
			steer_cmd->stamp = this->now();

			const float cmd_tire_angle = command->lateral.steering_tire_angle;
			if(cmd_tire_angle > 0)
			{
				steer_cmd->steer_cmd = cmd_tire_angle * tire_rad_to_steering_can_value_left_slope_ +
					tire_rad_to_steering_can_value_left_intercept_ + steer_input_center_ + steer_input_correction_send_;
				//RCLCPP_INFO(this->get_logger(), "left %d,%f,%lf,%lf",steer_cmd->steer_cmd, cmd_tire_angle, tire_rad_to_steering_can_value_left_slope_, tire_rad_to_steering_can_value_left_intercept_);
			}
			else
			{
				steer_cmd->steer_cmd = cmd_tire_angle * tire_rad_to_steering_can_value_right_slope_ +
					tire_rad_to_steering_can_value_right_intercept_ + steer_input_center_ + steer_input_correction_send_;
				//RCLCPP_INFO(this->get_logger(), "right %d,%f,%lf,%lf",steer_cmd->steer_cmd, cmd_tire_angle, tire_rad_to_steering_can_value_right_slope_, tire_rad_to_steering_can_value_right_intercept_);
			}
			steer_cmd->steer_cmd = std::min(std::max(steer_cmd->steer_cmd, steer_input_right_min_), steer_input_left_max_);

			pub_steer_cmd_->publish(std::move(steer_cmd));
		}

		//横滑り角のコールバック
		void callbackBeta(const std_msgs::msg::Bool::ConstSharedPtr msg)
		{
			beta_ = msg->data;
		}

	public:
		CarAutoOperationSteer(const rclcpp::NodeOptions options)
			: rclcpp::Node("car_auto_operation_steer", options)
		{
			this->declare_parameter<double>("tire_rad_to_steering_can_value_left_slope", 0.0);
			this->declare_parameter<double>("tire_rad_to_steering_can_value_left_intercept", 0.0);
			this->declare_parameter<double>("tire_rad_to_steering_can_value_right_slope", 0.0);
			this->declare_parameter<double>("tire_rad_to_steering_can_value_right_intercept", 0.0);
			this->declare_parameter<int16_t>("steer_input_left_max", 1941);
			this->declare_parameter<int16_t>("steer_input_right_min", 0);
			this->declare_parameter<int16_t>("steer_input_center", 1024);
			this->declare_parameter<int16_t>("steer_input_correction_send", 0);
			this->declare_parameter<bool>("use_beta", true);

			tire_rad_to_steering_can_value_left_slope_ = get_parameter("tire_rad_to_steering_can_value_left_slope").as_double();
			tire_rad_to_steering_can_value_left_intercept_ = get_parameter("tire_rad_to_steering_can_value_left_intercept").as_double();
			tire_rad_to_steering_can_value_right_slope_ = get_parameter("tire_rad_to_steering_can_value_right_slope").as_double();
			tire_rad_to_steering_can_value_right_intercept_ = get_parameter("tire_rad_to_steering_can_value_right_intercept").as_double();
			steer_input_left_max_ = static_cast<int16_t>(get_parameter("steer_input_left_max").as_int());
			steer_input_right_min_ = static_cast<int16_t>(get_parameter("steer_input_right_min").as_int());
			steer_input_center_ = static_cast<int16_t>(get_parameter("steer_input_center").as_int());
			steer_input_correction_send_ = static_cast<int16_t>(get_parameter("steer_input_correction_send").as_int());
			use_beta_ = get_parameter("use_beta").as_bool();

			RCLCPP_INFO(this->get_logger(), "tire_rad_to_steering_can_value_left_slope,%lf", tire_rad_to_steering_can_value_left_slope_);
			RCLCPP_INFO(this->get_logger(), "tire_rad_to_steering_can_value_left_intercept,%lf", tire_rad_to_steering_can_value_left_intercept_);
			RCLCPP_INFO(this->get_logger(), "tire_rad_to_steering_can_value_right_slope,%lf", tire_rad_to_steering_can_value_right_slope_);
			RCLCPP_INFO(this->get_logger(), "tire_rad_to_steering_can_value_right_intercept,%lf", tire_rad_to_steering_can_value_right_intercept_);
			RCLCPP_INFO(this->get_logger(), "steer_input_left_max,%d", steer_input_left_max_);
			RCLCPP_INFO(this->get_logger(), "steer_input_right_min,%d", steer_input_right_min_);
			RCLCPP_INFO(this->get_logger(), "steer_input_center,%d", steer_input_center_);
			RCLCPP_INFO(this->get_logger(), "steer_input_correction_send,%d", steer_input_correction_send_);
			RCLCPP_INFO(this->get_logger(), "steer_input_center+steer_input_correction_send,%d", steer_input_center_+steer_input_correction_send_);

			sub_control_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>("autoware_steer_cmd", rclcpp::QoS(1),
				std::bind(&CarAutoOperationSteer::callbackCommand, this, std::placeholders::_1));
			if(use_beta_) sub_beta_ = create_subscription<std_msgs::msg::Bool>("beta", rclcpp::QoS(1),
					std::bind(&CarAutoOperationSteer::callbackBeta, this, std::placeholders::_1));

			pub_steer_cmd_ = this->create_publisher<wada_vmc_msgs::msg::SteerCmd>("steer_cmd",rclcpp::QoS(1));
		}
	};
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<car_auto_operation::CarAutoOperationSteer> node = std::make_shared<car_auto_operation::CarAutoOperationSteer>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
