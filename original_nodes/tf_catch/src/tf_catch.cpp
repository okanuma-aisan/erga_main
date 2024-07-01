#include <rclcpp/rclcpp.hpp>
//#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/float64_multi_array.hpp>

class TFCatch : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

	void timerCallback()
	{
		tf2::BufferCore tf_buffer;
  		tf2_ros::TransformListener tf_listener(tf_buffer);
		rclcpp::sleep_for(std::chrono::milliseconds(100));
		try
		{
			geometry_msgs::msg::TransformStamped transform = tf_buffer.lookupTransform("map", "camera6_optical_link", tf2::TimePointZero);
			tf2::Transform tf_transform;
			//tf_transform.setOrigin(tf2::Vector3(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z));
			tf2::Quaternion qua = tf2::Quaternion(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
			double roll, pitch ,yaw;
			tf2::Matrix3x3(qua).getRPY(roll, pitch, yaw);
			RCLCPP_INFO(this->get_logger(), "%lf,%lf,%lf,%lf,%lf,%lf",
			transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z,
			roll, pitch, yaw);
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
		}
		
		/*rclcpp::Time start_time = this->now();
		rclcpp::Time end_time = this->now();
		rcl_time_point_value_t diff_time = end_time.nanoseconds() - start_time.nanoseconds();
		RCLCPP_INFO(this->get_logger(), "%ld", diff_time);*/

		/*static double a = 0;
		clock_t start_time = clock();
		std_msgs::msg::Float64MultiArray datas;
		for(int i=0; i<100000; i++) datas.data.push_back(a);
		clock_t end_time = clock();
		pub_->publish(datas);
		std::cout << (end_time - start_time) / (double)CLOCKS_PER_SEC << std::endl;
		//RCLCPP_INFO(this->get_logger(), "%lf", (end_time - start_time) / (double)CLOCKS_PER_SEC);
		a++;*/
	}
public:
	TFCatch() : rclcpp::Node("TFCatch")
	{
		tf2::Quaternion qua(0.5, -0.5, 0.5, -0.5);
		double roll, pitch, yaw;
		tf2::Matrix3x3(qua).getRPY(roll, pitch, yaw);
		RCLCPP_INFO(get_logger(), "%e,%e,%e", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
		timer_ = rclcpp::create_timer(
    	this, get_clock(), rclcpp::Duration(0, 1E8), std::bind(&TFCatch::timerCallback, this));

		/*const rclcpp::QoSInitialization qos_init(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1);
  		rclcpp::QoS qos(qos_init, rmw_qos_profile_sensor_data);

		RCLCPP_INFO(this->get_logger(), "depth,%ld", qos.depth());

		switch(qos.history())
		{
		case rclcpp::HistoryPolicy::KeepAll:
			RCLCPP_INFO(this->get_logger(), "history,KEEPALL");
			break;
		case rclcpp::HistoryPolicy::KeepLast:
			RCLCPP_INFO(this->get_logger(), "history,KEEPLAST");
			break;
		case rclcpp::HistoryPolicy::SystemDefault:
			RCLCPP_INFO(this->get_logger(), "history,SystemDefault");
			break;
		case rclcpp::HistoryPolicy::Unknown:
			RCLCPP_INFO(this->get_logger(), "history,Unknown");
			break;
		}

		switch(qos.reliability())
		{
		case rclcpp::ReliabilityPolicy::BestEffort:
			RCLCPP_INFO(this->get_logger(), "reliability,BestEffort");
			break;
		case rclcpp::ReliabilityPolicy::Reliable:
			RCLCPP_INFO(this->get_logger(), "reliability,Reliable");
			break;
		case rclcpp::ReliabilityPolicy::SystemDefault:
			RCLCPP_INFO(this->get_logger(), "reliability,SystemDefault");
			break;
		case rclcpp::ReliabilityPolicy::Unknown:
			RCLCPP_INFO(this->get_logger(), "reliability,Unknown");
			break;
		}

		switch(qos.durability())
		{
		case rclcpp::DurabilityPolicy::SystemDefault:
			RCLCPP_INFO(this->get_logger(), "durability,SystemDefault");
			break;
		case rclcpp::DurabilityPolicy::TransientLocal:
			RCLCPP_INFO(this->get_logger(), "durability,TransientLocal");
			break;
		case rclcpp::DurabilityPolicy::Unknown:
			RCLCPP_INFO(this->get_logger(), "durability,Unknown");
			break;
		case rclcpp::DurabilityPolicy::Volatile:
			RCLCPP_INFO(this->get_logger(), "durability,Volatile");
			break;
		}*/

		rclcpp::QoSInitialization qos_init(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5);
		rclcpp::QoS custom_qos(qos_init, rmw_qos_profile_sensor_data);
		pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/aaa", custom_qos);
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<TFCatch> tf_catch = std::make_shared<TFCatch>();
	rclcpp::spin(tf_catch);
	rclcpp::shutdown();
	return 0;
}