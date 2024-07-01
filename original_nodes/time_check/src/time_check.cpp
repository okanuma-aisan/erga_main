#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class TimerCheck : public rclcpp::Node
{
private:
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub1_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;

	rclcpp::Time pointcloud_time_;

	void callbackSub1(const sensor_msgs::msg::PointCloud2::ConstSharedPtr data)
	{
		if(read_pointcloud_ == false) return;
		rclcpp::Time navtime = data->header.stamp;
		rclcpp::Duration rostimediff = navtime - pointcloud_time_;
		double timediff = rostimediff.seconds() + rostimediff.nanoseconds() * 1E-9;
		RCLCPP_INFO(this->get_logger(), "timediff,%lf", timediff);
		//std::cout << "a" << std::endl;
	}

	bool read_pointcloud_ = false;
	void callbackSub2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr data)
	{
		//std::cout << "b" << std::endl;
		read_pointcloud_ = true;
		pointcloud_time_ = data->header.stamp;
	}
public:
	TimerCheck()  : rclcpp::Node("TimerCheck")
	{
		sub1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    		"/sensing/lidar/right/pointcloud_raw_ex", rclcpp::SensorDataQoS(),
    		std::bind(&TimerCheck::callbackSub1, this, std::placeholders::_1));
		sub2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    		"/sensing/lidar/top/pointcloud_raw_ex", rclcpp::SensorDataQoS(),
    		std::bind(&TimerCheck::callbackSub2, this, std::placeholders::_1));
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<TimerCheck> check = std::make_shared<TimerCheck>();
	rclcpp::spin(check);
	rclcpp::shutdown();
	return 0;
}