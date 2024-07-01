#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

class ImuHikaku : public rclcpp::Node
{
private:
	rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_imu1_, sub_imu2_;

	void callbackImu1(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
	{

	}

	void callbackImu2(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
	{

	}
public:
	ImuHikaku()
		: rclcpp::Node("imu_hikaku")
	{
		sub_imu1_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/aaa", rclcpp::SensorDataQoS(),
			std::bind(&ImuHikaku::callbackImu1, this, std::placeholders::_1));
		sub_imu2_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/bbb", rclcpp::SensorDataQoS(),
			std::bind(&ImuHikaku::callbackImu2, this, std::placeholders::_1));
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	return 0;
}