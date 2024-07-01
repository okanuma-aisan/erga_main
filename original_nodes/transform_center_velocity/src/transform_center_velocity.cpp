// IMUから車両重心速度をpublishするノード
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class TransformCnterVelocity : public rclcpp::Node
{
private: // params
	double wheel_base_ = 1.0;			 // 5.3;//wheel_base
	double car_center_from_imu_x_ = 1.0; // 1.83584337349398;//imuからの重心位置(x)
	double car_center_from_imu_y_ = 1.0; // 3.46415662650602;//imuからの重心位置(y)
	std::string output_frame_;			 // 車両重心速度情報のフレームID

private: // ros2 subscriber
	rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_gyro_vel_; // ジャイロ速度
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_tiwst_;						   // imu情報

private: // ros2 publisher
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_center_twist_; // 車両重心速度
	rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_center_twist_cov_; // 車両重心速度

private: // ros tf
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

private: // subscribeした情報
	bool is_read_gyro_vel_ = false;
	geometry_msgs::msg::TwistWithCovarianceStamped gyro_vel_; // ジャイロ速度

private: // ros2 callback
	// ジャイロ速度
	void callbackGyroVel(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr gyro_vel)
	{
		gyro_vel_ = *gyro_vel;
		is_read_gyro_vel_ = true;
	}

	// imu情報
	void callbackImuTwist(const sensor_msgs::msg::Imu::ConstSharedPtr imu_tiwst)
	{
		if (is_read_gyro_vel_)
		{
			double vx = gyro_vel_.twist.twist.linear.x;
			double vy = gyro_vel_.twist.twist.linear.y;
			double omega = imu_tiwst->angular_velocity.z;//imu_twist->angular_velocity.z;
			double L = wheel_base_;
			double theta = std::atan2(car_center_from_imu_y_, car_center_from_imu_x_);
			double car_vel_x = vx - omega * L * (-std::cos(theta));
			double car_vel_y = vy - omega * L * (std::sin(theta));

			geometry_msgs::msg::TransformStamped::SharedPtr tf_base2imu_ptr =
				std::make_shared<geometry_msgs::msg::TransformStamped>();
			getTransform(output_frame_, imu_tiwst->header.frame_id, tf_base2imu_ptr);

			geometry_msgs::msg::Vector3Stamped angular_velocity;
			angular_velocity.header = imu_tiwst->header;
			angular_velocity.vector = imu_tiwst->angular_velocity;

			geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
			transformed_angular_velocity.header = tf_base2imu_ptr->header;
			tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_base2imu_ptr);

			geometry_msgs::msg::TwistStamped msg_vel;
			msg_vel.header.frame_id = output_frame_;
			msg_vel.header.stamp = this->now();
			msg_vel.twist.linear.x = car_vel_x;
			msg_vel.twist.linear.y = car_vel_y;
			msg_vel.twist.linear.z = gyro_vel_.twist.twist.linear.z;
			msg_vel.twist.angular.x = transformed_angular_velocity.vector.x;
			msg_vel.twist.angular.y = transformed_angular_velocity.vector.y;
			msg_vel.twist.angular.z = transformed_angular_velocity.vector.z;
			pub_center_twist_->publish(msg_vel);

			geometry_msgs::msg::TwistWithCovarianceStamped msg_vel_cov;
			msg_vel_cov.header = msg_vel.header;
			msg_vel_cov.twist.twist = msg_vel.twist;
			msg_vel_cov.twist.twist.angular.x = transformed_angular_velocity.vector.x;
			msg_vel_cov.twist.twist.angular.y = transformed_angular_velocity.vector.y;
			msg_vel_cov.twist.twist.angular.z = transformed_angular_velocity.vector.z;
			pub_center_twist_cov_->publish(msg_vel_cov);
		}
	}

private: // その他
	bool getTransform(const std::string &target_frame, const std::string &source_frame,
					  const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr)
	{
		if (target_frame == source_frame)
		{
			transform_stamped_ptr->header.stamp = this->get_clock()->now();
			transform_stamped_ptr->header.frame_id = target_frame;
			transform_stamped_ptr->child_frame_id = source_frame;
			transform_stamped_ptr->transform.translation.x = 0.0;
			transform_stamped_ptr->transform.translation.y = 0.0;
			transform_stamped_ptr->transform.translation.z = 0.0;
			transform_stamped_ptr->transform.rotation.x = 0.0;
			transform_stamped_ptr->transform.rotation.y = 0.0;
			transform_stamped_ptr->transform.rotation.z = 0.0;
			transform_stamped_ptr->transform.rotation.w = 1.0;
			return true;
		}

		try
		{
			*transform_stamped_ptr =
				tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_WARN(this->get_logger(), "%s", ex.what());
			RCLCPP_ERROR(this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

			transform_stamped_ptr->header.stamp = this->get_clock()->now();
			transform_stamped_ptr->header.frame_id = target_frame;
			transform_stamped_ptr->child_frame_id = source_frame;
			transform_stamped_ptr->transform.translation.x = 0.0;
			transform_stamped_ptr->transform.translation.y = 0.0;
			transform_stamped_ptr->transform.translation.z = 0.0;
			transform_stamped_ptr->transform.rotation.x = 0.0;
			transform_stamped_ptr->transform.rotation.y = 0.0;
			transform_stamped_ptr->transform.rotation.z = 0.0;
			transform_stamped_ptr->transform.rotation.w = 1.0;
			return false;
		}
		return true;
	}

public:
	TransformCnterVelocity(const rclcpp::NodeOptions node_options)
		: rclcpp::Node("transform_center_velocity", node_options)
		, tf_buffer_(this->get_clock())
		, tf_listener_(tf_buffer_)
	{
		this->declare_parameter<double>("wheel_base", 1.0);
		this->declare_parameter<double>("car_center_from_imu_x", 1.0);
		this->declare_parameter<double>("car_center_from_imu_y", 1.0);
		this->declare_parameter<std::string>("output_frame", "base_link");
		wheel_base_ = this->get_parameter("wheel_base").as_double();
		car_center_from_imu_x_ = this->get_parameter("car_center_from_imu_x").as_double();
		car_center_from_imu_y_ = this->get_parameter("car_center_from_imu_y").as_double();
		output_frame_ = this->get_parameter("output_frame").as_string();
		RCLCPP_INFO(this->get_logger(), "wheel_base:%lf", wheel_base_);
		RCLCPP_INFO(this->get_logger(), "car_center_from_imu_x:%lf", car_center_from_imu_x_);
		RCLCPP_INFO(this->get_logger(), "car_center_from_imu_y:%lf", car_center_from_imu_y_);
		RCLCPP_INFO(this->get_logger(), "output_frame:%s", output_frame_.c_str());

		sub_gyro_vel_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("gyro_vel", rclcpp::QoS(1),
			std::bind(&TransformCnterVelocity::callbackGyroVel, this, std::placeholders::_1));
		sub_imu_tiwst_ = create_subscription<sensor_msgs::msg::Imu>("imu_twist", rclcpp::QoS(1),
			std::bind(&TransformCnterVelocity::callbackImuTwist, this, std::placeholders::_1));
		pub_center_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("car_center_velocity", rclcpp::QoS(1));
		pub_center_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("car_center_velocity_cov", rclcpp::QoS(1));
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<TransformCnterVelocity> nodes = std::make_shared<TransformCnterVelocity>(node_options);
	rclcpp::spin(nodes);
	rclcpp::shutdown();
	return 0;
}