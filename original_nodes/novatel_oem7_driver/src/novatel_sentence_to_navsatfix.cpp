#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <orig_system_msgs/msg/date.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

double dig2rad(double dig)
{
	while(dig <0 || dig >=360){
		if(dig<0) dig+=360;
	  else if(dig>=360) dig-=360;
	}
	return dig*M_PI/180.0;
}

void sentence_split(const std::string &sentence_string, std::vector<std::string> &split_str)
{
	std::vector<std::string> str_vec_ptr;
	std::string token1;
	std::stringstream ss1(sentence_string);

	while (getline(ss1, token1, ','))
	{
		std::string token2;
		std::stringstream ss2(token1);
		while(getline(ss2, token2, ';'))
		{
			std::string token3;
			std::stringstream ss3(token2);
			while(getline(ss3, token3, '*'))
				split_str.push_back(token3);
		}
	}
}

class NovatelSentenceToNavSatFix : public rclcpp::Node
{
private:
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_sentence_;//novatelセンテンス
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nav_sat_fix_;//センテンス(BESTPOS)から作成した位置
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nav_sat_fix_rtkpos_;//センテンス(RTKPOS)から作成した位置
	rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr pub_gnss_rotation_;//センテンスから作成した向き
	rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_fix_vel_;//センテンスから作成したxyz速度
	rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_surface_speed_;//センテンスから作成した車両速度 sqrt(x*x+y*y+z*z)
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;//センテンスから作成した、novatelと関連付けられているIMU数値
	rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_odometry_;//センテンスから作成したオドメトリー
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_pos_var_;//センテンスから作成した緯度経度の分散
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vel_var_;//センテンスから作成した速度の分散
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_ang_var_;//センテンスから作成した角度の分散
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_velocity_angle_;//センテンスから作成した角速度
	rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr pub_gpgga_;//センテンスから作成したGPGGA
	rclcpp::Publisher<orig_system_msgs::msg::Date>::SharedPtr pub_date_;//センテンスから作成した時刻
	rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_vel_report_;//autoware用速度レポート
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_solution_;//受信精度

	double inspva_lat_var_;//inspvaから作成した世界座標系の緯度の分散値
	double inspva_lon_var_;//inspvaから作成した世界座標系の軽度の分散値
	double inspva_alt_var_;//inspvaから作成した世界座標系の高度の分散値
	double inspva_north_vel_var_;//inspvaから作成した、世界座標系を基準とした北方向速度の分散値
	double inspva_east_vel_var_;//inspvaから作成した、世界座標系を基準とした東方向速度の分散値
	double inspva_up_vel_var_;//inspvaから作成した、世界座標系を基準とした上方向速度の分散値
	double inspva_roll_var_;//inspvaから作成した、世界座標系を基準としたroll角の分散値
	double inspva_pitch_var_;//inspvaから作成した、世界座標系を基準としたpitch角の分散値
	double inspva_yaw_var_;//inspvaから作成した、世界座標系を基準としたyaw角の分散値
	double surface_speed_;//novatelから得られた現在速度

	double acceleration_scale_factor_;//novatelと関連付けられているIMUの加速度スケールファクター RAWIMUセンテンスから得られる数値にこの数値をかけたものが実際の加速度になる
	double gyroscope_scale_factor_;//novatelと関連付けられているIMUの角速度スケールファクター RAWIMUセンテンスから得られる数値にこの数値をかけたものが実際の加速度になる
	int imu_hz_;//novatelと関連付けられているIMUの処理周期

	void callbackSentence(const std_msgs::msg::String::ConstSharedPtr msg)
	{
		const rclcpp::Time rosnowtime = this->now();
		const std::string frame_name = "gnss_link";

		std::vector<std::string> split_str;
		sentence_split(msg->data, split_str);
//		RCLCPP_INFO(this->get_logger(), "%s", split_str[0]);
		if(split_str.empty()) return;

/*		std::cout << split_str[0] << "," << split_str.size() << std::endl;
		for(int i=0; i<split_str.size(); i++)
			std::cout << split_str[i] << ",";
		std::cout << std::endl;
*/
		if(split_str[0] ==  "#BESTPOSA" && split_str.size() == 32)
		{
			sensor_msgs::msg::NavSatFix nav_sat_fix;
			nav_sat_fix.header.stamp = rosnowtime;
			nav_sat_fix.header.frame_id = frame_name;
			nav_sat_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
			nav_sat_fix.status.service = 15; //not use 2022_09_05 //sensor_msgs::msg::NavSatStatus::SERVICE_GPS | sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS | sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO | sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
			nav_sat_fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

			//for(std::string str : split_str)
			//	RCLCPP_INFO(this->get_logger(), "%s", str.c_str());
			try
			{
				nav_sat_fix.latitude = std::stod(split_str[12]);
				nav_sat_fix.longitude = std::stod(split_str[13]);
				nav_sat_fix.altitude = std::stod(split_str[14]);
				for(double &cov : nav_sat_fix.position_covariance)
					cov = 0;
				double lat_var = std::stod(split_str[17]);
				double lon_var = std::stod(split_str[18]);
				double alt_var = std::stod(split_str[19]);
				nav_sat_fix.position_covariance[0] = lat_var * lat_var;
				nav_sat_fix.position_covariance[4] = lon_var * lon_var;
				nav_sat_fix.position_covariance[8] = alt_var * alt_var;

				pub_nav_sat_fix_->publish(nav_sat_fix);

			}
			catch(std::invalid_argument e)
			{
			}
			catch(std::out_of_range e)
			{
			}
		}
		else if(split_str[0] == "#RTKPOSA" && split_str.size() == 32)
		{
			sensor_msgs::msg::NavSatFix nav_sat_fix;
			nav_sat_fix.header.stamp = rosnowtime;
			nav_sat_fix.header.frame_id = frame_name;
			nav_sat_fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
			nav_sat_fix.status.service = 15; //not use 2022_09_05 //sensor_msgs::msg::NavSatStatus::SERVICE_GPS | sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS | sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO | sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
			nav_sat_fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

			//for(std::string str : split_str)
			//	RCLCPP_INFO(this->get_logger(), "%s", str.c_str());
			try
			{
				nav_sat_fix.latitude = std::stod(split_str[12]);
				nav_sat_fix.longitude = std::stod(split_str[13]);
				nav_sat_fix.altitude = std::stod(split_str[14]);
				for(double &cov : nav_sat_fix.position_covariance)
					cov = 0;
				double lat_var = std::stod(split_str[17]);
				double lon_var = std::stod(split_str[18]);
				double alt_var = std::stod(split_str[19]);
				nav_sat_fix.position_covariance[0] = lat_var * lat_var;
				nav_sat_fix.position_covariance[4] = lon_var * lon_var;
				nav_sat_fix.position_covariance[8] = alt_var * alt_var;

				pub_nav_sat_fix_rtkpos_->publish(nav_sat_fix);
			}
			catch(std::invalid_argument e)
			{
			}
			catch(std::out_of_range e)
			{
			}
		}
		else if(split_str[0] == "#INSPVAXA" && split_str.size() == 34)
		{
			geometry_msgs::msg::TwistWithCovarianceStamped fix_vel;
			fix_vel.header.stamp = rosnowtime;
			fix_vel.header.frame_id = frame_name;

			geometry_msgs::msg::TwistWithCovarianceStamped surface_speed;
			surface_speed.header.stamp = rosnowtime;
			surface_speed.header.frame_id = "base_link";

			try
			{
				fix_vel.twist.twist.linear.x = std::stod(split_str[16]);// * 100;
				fix_vel.twist.twist.linear.y = std::stod(split_str[17]);// * 100;
				fix_vel.twist.twist.linear.z = std::stod(split_str[18]);// * 100;
				for(double &cov : fix_vel.twist.covariance)
					cov = 0;

				surface_speed.twist.twist.linear.x = std::sqrt(
					fix_vel.twist.twist.linear.x * fix_vel.twist.twist.linear.x +
					fix_vel.twist.twist.linear.y * fix_vel.twist.twist.linear.y +
					fix_vel.twist.twist.linear.z * fix_vel.twist.twist.linear.z);
				if(surface_speed.twist.twist.linear.x < 0.05) surface_speed.twist.twist.linear.x = 0;

				double inspva_roll= dig2rad(std::stod(split_str[19]));
				double inspva_pitch= -dig2rad(std::stod(split_str[20]));
				double inspva_yaw = dig2rad(std::stod(split_str[21]));

				//車両の向きと速度の向きから前進か後退を判断
				tf2::Quaternion car_qua;
				car_qua.setRPY(inspva_roll, inspva_pitch, inspva_yaw);
				double velocity_angle = std::atan2(fix_vel.twist.twist.linear.y, fix_vel.twist.twist.linear.x);
				tf2::Quaternion vel_qua;
				vel_qua.setRPY(0, 0, velocity_angle);
				double vel_roll, vel_pitch, vel_yaw;
				tf2::Matrix3x3(vel_qua).getRPY(vel_roll, vel_pitch, vel_yaw);
				tf2::Quaternion nasu = car_qua * vel_qua.inverse();
				double nasu_roll, nasu_pitch, nasu_yaw;
				tf2::Matrix3x3(nasu).getRPY(nasu_roll, nasu_pitch, nasu_yaw);
				if(std::abs(nasu_yaw) > 90*M_PI/180.0) surface_speed.twist.twist.linear.x *= -1;

				//x軸に対する角度をy軸に対する角度に変換 yaw角を求めるため
				double inspva_yaw_x = cos(inspva_yaw), inspva_yaw_y = sin(inspva_yaw);
				inspva_yaw = atan2(inspva_yaw_x, inspva_yaw_y);
				tf2::Quaternion car_qua2;
				car_qua2.setRPY(inspva_roll, inspva_pitch, inspva_yaw);

				surface_speed.twist.covariance[0] = 0.062;
				surface_speed.twist.covariance[7] = 0.062;
				surface_speed.twist.covariance[14] = 0.062;
				surface_speed.twist.covariance[21] = -1;
				pub_surface_speed_->publish(surface_speed);
				surface_speed_ = surface_speed.twist.twist.linear.x;

				autoware_sensing_msgs::msg::GnssInsOrientationStamped gnss_ori;
				gnss_ori.header.stamp = rosnowtime;
				gnss_ori.header.frame_id = frame_name;
				gnss_ori.orientation.orientation = tf2::toMsg(car_qua2);//autowareが扱う車両向きはy軸
				gnss_ori.orientation.rmse_rotation_x = 0;
				gnss_ori.orientation.rmse_rotation_y = 0;
				gnss_ori.orientation.rmse_rotation_z = 0;
				pub_gnss_rotation_->publish(gnss_ori);

				inspva_lat_var_ = std::stod(split_str[22]);
				inspva_lon_var_ = std::stod(split_str[23]);
				inspva_alt_var_ = std::stod(split_str[24]);
				inspva_north_vel_var_ = std::stod(split_str[25]);
				inspva_east_vel_var_ = std::stod(split_str[26]);
				inspva_up_vel_var_ = std::stod(split_str[27]);
				inspva_roll_var_ = std::stod(split_str[28]);
				inspva_pitch_var_ = std::stod(split_str[29]);
				inspva_yaw_var_ = std::stod(split_str[30]);
				fix_vel.twist.covariance[0] = inspva_north_vel_var_ * inspva_north_vel_var_;
				fix_vel.twist.covariance[7] = inspva_east_vel_var_ * inspva_east_vel_var_;
				fix_vel.twist.covariance[14] = inspva_up_vel_var_ * inspva_up_vel_var_;
				fix_vel.twist.covariance[21] = -1;
				pub_fix_vel_->publish(fix_vel);

				std_msgs::msg::String str;
				std::stringstream ss;
				ss << vel_yaw*180/M_PI << "," << inspva_yaw*180/M_PI << "," << nasu_yaw*180/M_PI;
				str.data = ss.str();
				pub_debug_velocity_angle_->publish(str);

				autoware_auto_vehicle_msgs::msg::VelocityReport msg_vel_report;
				msg_vel_report.header.frame_id = "base_link";
				msg_vel_report.header.stamp = rosnowtime;
				msg_vel_report.longitudinal_velocity = surface_speed.twist.twist.linear.x;
				msg_vel_report.lateral_velocity = 0;
				msg_vel_report.heading_rate = 0;
				pub_vel_report_->publish(msg_vel_report);

				std_msgs::msg::String msg_sol;
				msg_sol.data = split_str[10];
				pub_solution_->publish(msg_sol);
			}
			catch(std::invalid_argument e)
			{
			}
			catch(std::out_of_range e)
			{
			}
		}
		else if(split_str[0] == "#INSPVAA" && split_str.size() == 23)
		{
			geometry_msgs::msg::TwistWithCovarianceStamped fix_vel;
			fix_vel.header.stamp = rosnowtime;
			fix_vel.header.frame_id = frame_name;

			geometry_msgs::msg::TwistWithCovarianceStamped surface_speed;
			surface_speed.header.stamp = rosnowtime;
			surface_speed.header.frame_id = "base_link";

			try
			{
				fix_vel.twist.twist.linear.x = std::stod(split_str[15]);
				fix_vel.twist.twist.linear.y = std::stod(split_str[16]);
				fix_vel.twist.twist.linear.z = std::stod(split_str[17]);
				fix_vel.twist.covariance[0] = inspva_north_vel_var_ * inspva_north_vel_var_;
				fix_vel.twist.covariance[7] = inspva_east_vel_var_ * inspva_east_vel_var_;
				fix_vel.twist.covariance[14] = inspva_up_vel_var_ * inspva_up_vel_var_;
				fix_vel.twist.covariance[21] = -1;
				pub_fix_vel_->publish(fix_vel);

				surface_speed.twist.twist.linear.x = std::sqrt(
					fix_vel.twist.twist.linear.x * fix_vel.twist.twist.linear.x +
					fix_vel.twist.twist.linear.y * fix_vel.twist.twist.linear.y +
					fix_vel.twist.twist.linear.z * fix_vel.twist.twist.linear.z);
				if(surface_speed.twist.twist.linear.x < 0.05) surface_speed.twist.twist.linear.x = 0;

				double inspva_roll= dig2rad(std::stod(split_str[18]));
				double inspva_pitch= -dig2rad(std::stod(split_str[19]));
				double inspva_yaw = dig2rad(std::stod(split_str[20]));

				//車両の向きと速度の向きから前進か後退を判断
				tf2::Quaternion car_qua;
				car_qua.setRPY(inspva_roll, inspva_pitch, inspva_yaw);
				double velocity_angle = std::atan2(fix_vel.twist.twist.linear.y, fix_vel.twist.twist.linear.x);
				tf2::Quaternion vel_qua;
				vel_qua.setRPY(0, 0, velocity_angle);
				tf2::Quaternion nasu = car_qua * vel_qua.inverse();
				double nasu_roll, nasu_pitch, nasu_yaw;
				tf2::Matrix3x3(nasu).getRPY(nasu_roll, nasu_pitch, nasu_yaw);
				if(std::abs(nasu_yaw) > 90*M_PI/180.0) surface_speed.twist.twist.linear.x *= -1;

				pub_surface_speed_->publish(surface_speed);
				surface_speed_ = surface_speed.twist.twist.linear.x;

				//x軸に対する角度をy軸に対する角度に変換 yaw角を求めるため
				double inspva_yaw_x = cos(inspva_yaw), inspva_yaw_y = sin(inspva_yaw);
				inspva_yaw = atan2(inspva_yaw_x, inspva_yaw_y);
				tf2::Quaternion car_qua2;
				car_qua2.setRPY(inspva_roll, inspva_pitch, inspva_yaw);

				autoware_sensing_msgs::msg::GnssInsOrientationStamped gnss_ori;
				gnss_ori.header.stamp = rosnowtime;
				gnss_ori.header.frame_id = frame_name;
				gnss_ori.orientation.orientation = tf2::toMsg(car_qua2);//autowareが扱う車両向きはy軸
				gnss_ori.orientation.rmse_rotation_x = 0;
				gnss_ori.orientation.rmse_rotation_y = 0;
				gnss_ori.orientation.rmse_rotation_z = 0;
				pub_gnss_rotation_->publish(gnss_ori);

				std_msgs::msg::String str;
				std::stringstream ss;
				double vel_roll, vel_pitch, vel_yaw;
				tf2::Matrix3x3(vel_qua).getRPY(vel_roll, vel_pitch, vel_yaw);
				ss << vel_yaw*180/M_PI << "," << inspva_yaw*180/M_PI << "," << nasu_yaw*180/M_PI;
				str.data = ss.str();
				pub_debug_velocity_angle_->publish(str);

				autoware_auto_vehicle_msgs::msg::VelocityReport msg_vel_report;
				msg_vel_report.header.frame_id = "base_link";
				msg_vel_report.header.stamp = rosnowtime;
				msg_vel_report.longitudinal_velocity = surface_speed.twist.twist.linear.x;
				msg_vel_report.lateral_velocity = 0;
				msg_vel_report.heading_rate = 0;
				pub_vel_report_->publish(msg_vel_report);
			}
			catch(std::invalid_argument e)
			{
			}
			catch(std::out_of_range e)
			{
			}
		}
		else if(split_str[0] == "#INSSTDEVA" && split_str.size() == 25)
		{
			try
			{
				inspva_lat_var_ = std::stod(split_str[10]);
				inspva_lon_var_ = std::stod(split_str[11]);
				inspva_alt_var_ = std::stod(split_str[12]);
				inspva_north_vel_var_ = std::stod(split_str[13]);
				inspva_east_vel_var_ = std::stod(split_str[14]);
				inspva_up_vel_var_ = std::stod(split_str[15]);
				inspva_roll_var_ = std::stod(split_str[16]);
				inspva_pitch_var_ = std::stod(split_str[17]);
				inspva_yaw_var_ = std::stod(split_str[18]);
			}
			catch(std::invalid_argument e)
			{
			}
			catch(std::out_of_range e)
			{
			}

			geometry_msgs::msg::Vector3Stamped pos_var, vel_var, ang_var;

			pos_var.header.stamp = rosnowtime;
			pos_var.header.frame_id = "gnss_link";
			pos_var.vector.x = inspva_lat_var_;
			pos_var.vector.y = inspva_lon_var_;
			pos_var.vector.z = inspva_alt_var_;
			pub_pos_var_->publish(pos_var);

			vel_var.header.stamp = rosnowtime;
			vel_var.header.frame_id = "gnss_link";
			vel_var.vector.x = inspva_north_vel_var_;
			vel_var.vector.y = inspva_east_vel_var_;
			vel_var.vector.z = inspva_up_vel_var_;
			pub_vel_var_->publish(vel_var);

			ang_var.header.stamp = rosnowtime;
			ang_var.header.frame_id = "gnss_link";
			ang_var.vector.x = inspva_roll_var_;
			ang_var.vector.y = inspva_pitch_var_;
			ang_var.vector.z = inspva_yaw_var_;
			pub_ang_var_->publish(ang_var);
		}
		else if(split_str[0] == "#RAWIMUA" && split_str.size() == 20)
		{		
			//PwrPak7D-E1 scale factor
			//https://docs.novatel.com/OEM7/Content/SPAN_Logs/RAWIMUS.htm

			//ISA100用スケールファクター設定 スケールファクター×HZ
			//const double acceleration_scale_factor = 2.0E-8 * 200;//2.0E-8;
			//const double gyroscope_scale_factor = 1.0E-9 * 200;//1.0E-9 * 6.83;

			//span用スケールファクター設定 スケールファクター×HZ
			//const double acceleration_scale_factor = std::pow(2.0, -29) * 100;//1.85E-7;
			//const double gyroscope_scale_factor = std::pow(2.0, -33) * 100;//1.85E-7;

			const double acc_scale_factor = acceleration_scale_factor_ * imu_hz_;
			const double gyro_scale_factor = gyroscope_scale_factor_ * imu_hz_;

			sensor_msgs::msg::Imu imu;
			imu.header.stamp = rosnowtime;
			imu.header.frame_id = "imu_link";

			try
			{
				imu.orientation.w = 1;
				imu.orientation.x = imu.orientation.y = imu.orientation.z = 0;
				for(double &v : imu.orientation_covariance) v = 0;
				imu.linear_acceleration.z = std::stod(split_str[13]) * acc_scale_factor;
				imu.linear_acceleration.y = -std::stod(split_str[14]) * acc_scale_factor;
				imu.linear_acceleration.x = std::stod(split_str[15]) * acc_scale_factor;
				for(double &v : imu.linear_acceleration_covariance) v = 0;
				imu.linear_acceleration_covariance[0] = 0.3*0.3;//inspva_north_vel_var_ * inspva_north_vel_var_;
				imu.linear_acceleration_covariance[4] = 0.3*0.3;//inspva_east_vel_var_ * inspva_east_vel_var_;
				imu.linear_acceleration_covariance[8] = 0.3*0.3;//inspva_up_vel_var_ * inspva_up_vel_var_;
				imu.angular_velocity.z = std::stod(split_str[16]) * gyro_scale_factor;
				imu.angular_velocity.y = -std::stod(split_str[17]) * gyro_scale_factor;
				imu.angular_velocity.x = std::stod(split_str[18]) * gyro_scale_factor;
				for(double &v : imu.angular_velocity_covariance) v = 0;
				imu.angular_velocity_covariance[0] = 0.3*0.3;//inspva_roll_var_ * inspva_roll_var_;
				imu.angular_velocity_covariance[4] = 0.3*0.3;//inspva_pitch_var_ * inspva_pitch_var_;
				imu.angular_velocity_covariance[8] = 0.3*0.3;//inspva_yaw_var_ * inspva_yaw_var_;

				pub_imu_->publish(imu);

				geometry_msgs::msg::TwistWithCovarianceStamped odom;
				odom.header.stamp = rosnowtime;
				odom.header.frame_id = "base_link";
				odom.twist.twist.linear.x = surface_speed_;
				odom.twist.twist.linear.y = 0;
				odom.twist.twist.linear.z = 0;
				odom.twist.twist.angular.x = imu.angular_velocity.x;
				odom.twist.twist.angular.y = imu.angular_velocity.y;
				odom.twist.twist.angular.z = imu.angular_velocity.z;
				odom.twist.covariance[0 * 3 + 0] = 0.03 * 0.03;
				odom.twist.covariance[1 * 3 + 1] = 0.03 * 0.03;
				odom.twist.covariance[2 * 3 + 2] = 0.03 * 0.03;
				pub_odometry_->publish(odom);
			}
			catch(std::invalid_argument e)
			{
			}
			catch(std::out_of_range e)
			{
			}
		}
		else if(split_str[0] == "#TIMEA" && split_str.size() == 22)
		{
			try
			{
				orig_system_msgs::msg::Date msg_date;
				msg_date.year = std::stoi(split_str.at(14));
				msg_date.month = std::stoi(split_str.at(15));
				msg_date.day = std::stoi(split_str.at(16));
				msg_date.hour = std::stoi(split_str.at(17));
				msg_date.min = std::stoi(split_str.at(18));
				msg_date.sec = std::stoi(split_str.at(19)) / (float)1000.0;
				pub_date_->publish(msg_date);
			}
			catch(std::invalid_argument e)
			{
			}
			catch(std::out_of_range e)
			{
			}
		}
		else if(split_str[0] == "$GPGGA" && split_str.size() == 16)
		{
			try
			{
				nmea_msgs::msg::Gpgga gpgga;
				gpgga.header.stamp = this->now();
				gpgga.message_id = "GPGGA";
				gpgga.utc_seconds = std::stof(split_str[1]);
				gpgga.lat = std::stod(split_str[2]);
				gpgga.lon = std::stod(split_str[4]);
				gpgga.lat_dir = split_str[3];
				gpgga.lon_dir = split_str[5];
				gpgga.gps_qual = std::stoi(split_str[6]);
				gpgga.num_sats = std::stoi(split_str[7]);
				gpgga.hdop = std::stof(split_str[8]);
				gpgga.alt = std::stof(split_str[9]);
				gpgga.altitude_units = split_str[10];
				gpgga.undulation = std::stof(split_str[11]);
				gpgga.undulation_units = split_str[12];
				gpgga.diff_age = static_cast<uint32_t>(std::stoi(split_str[13]));
				gpgga.station_id = split_str[14];
				pub_gpgga_->publish(gpgga);
			}
			catch(std::invalid_argument e)
			{
			}
			catch(std::out_of_range e)
			{
			}
		}
	}
public:
	NovatelSentenceToNavSatFix(const std::string & node_name, const rclcpp::NodeOptions & options)
		: Node(node_name, options)
		, inspva_lat_var_(0)
		, inspva_lon_var_(0)
		, inspva_alt_var_(0)
		, inspva_north_vel_var_(0)
		, inspva_east_vel_var_(0)
		, inspva_up_vel_var_(0)
		, inspva_roll_var_(0)
		, inspva_pitch_var_(0)
		, inspva_yaw_var_(0)
		, surface_speed_(0)
	{
		acceleration_scale_factor_ = this->declare_parameter<double>("acceleration_scale_factor", 0.0);
		gyroscope_scale_factor_ = this->declare_parameter<double>("gyroscope_scale_factor", 0.0);
		imu_hz_ = this->declare_parameter<int>("imu_hz", 0);
		//RCLCPP_INFO(this->get_logger(), "factor,%e,%e,%u", acceleration_scale_factor_, gyroscope_scale_factor_, imu_hz_);

		sub_sentence_ = create_subscription<std_msgs::msg::String>("sentence", rclcpp::SensorDataQoS(),
			std::bind(&NovatelSentenceToNavSatFix::callbackSentence, this, std::placeholders::_1));

		pub_nav_sat_fix_ = create_publisher<sensor_msgs::msg::NavSatFix>("nav_sat_fix", rclcpp::QoS{1});
		pub_nav_sat_fix_rtkpos_ = create_publisher<sensor_msgs::msg::NavSatFix>("nav_sat_fix_rtkpos", rclcpp::QoS{1});
		pub_gnss_rotation_ = create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>("/autoware_orientation", rclcpp::QoS{1});
		pub_fix_vel_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("fixed_velocity", rclcpp::QoS{1});
		pub_surface_speed_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("surface_speed", rclcpp::QoS{1});
		pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::QoS{1});
		pub_odometry_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("odometry", rclcpp::QoS{1});
		pub_pos_var_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("pos_var", rclcpp::QoS{1});
		pub_vel_var_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("vel_var", rclcpp::QoS{1});
		pub_ang_var_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("ang_var", rclcpp::QoS{1});
		pub_gpgga_ = create_publisher<nmea_msgs::msg::Gpgga>("gpgga", rclcpp::QoS{1});
		pub_date_ = create_publisher<orig_system_msgs::msg::Date>("date", rclcpp::QoS{1});
		pub_debug_velocity_angle_ = create_publisher<std_msgs::msg::String>("debug/vel_angle", rclcpp::QoS{1});
		pub_vel_report_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("velocity_report", rclcpp::QoS{1});
		pub_solution_ = create_publisher<std_msgs::msg::String>("solution", rclcpp::QoS{1});
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<NovatelSentenceToNavSatFix> transformer = std::make_shared<NovatelSentenceToNavSatFix>("novatel_sentence_to_navsatfix", node_options);
	rclcpp::spin(transformer);
	rclcpp::shutdown();
	return 0;
}
