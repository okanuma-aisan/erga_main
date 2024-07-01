#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

class AAA : public rclcpp::Node
{
private:
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_aaa_;

	void callbackAAA(const std_msgs::msg::Empty)
	{
		std::vector<double> buf(100);
		RCLCPP_INFO(get_logger(), "%lf", buf[100000]);
	}
public:
	AAA(const rclcpp::NodeOptions &node_options)
		: rclcpp::Node("valgrind_sample", node_options)
	{
		sub_aaa_ = create_subscription<std_msgs::msg::Empty>("aaa", rclcpp::QoS(1),
			std::bind(&AAA::callbackAAA, this, std::placeholders::_1));
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<AAA> node = std::make_shared<AAA>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
