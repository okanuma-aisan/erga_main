#include <rclcpp/rclcpp.hpp>
#include <auto_permission_msgs/msg/auto_permission.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <report_selector_msgs/msg/report_select.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>

namespace auto_permission
{
	//ユークリッド距離を計算
	double euclidDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
	{
		double x = p1.x - p2.x;
		double y = p1.y - p2.y;
		double z = p1.z - p2.z;
		return std::sqrt(x*x + y*y + z*z);
	}

	//なす角(RPY)を計算
	geometry_msgs::msg::Vector3 formedAngular(const geometry_msgs::msg::Quaternion way_qua, const geometry_msgs::msg::Quaternion pos_qua)
	{
		tf2::Quaternion current_q(pos_qua.x, pos_qua.y, pos_qua.z,
			pos_qua.w);

		tf2::Quaternion way_q(way_qua.x, way_qua.y, way_qua.z,
			way_qua.w);
		tf2::Matrix3x3 way_m(way_q);
		double way_yaw, way_roll, way_pitch;
		way_m.getRPY(way_roll, way_pitch, way_yaw);

		tf2::Quaternion fa_q = way_q * current_q.inverse();
		tf2::Matrix3x3 fa_m(fa_q);

		geometry_msgs::msg::Vector3 ret;
		fa_m.getRPY(ret.x, ret.y, ret.z);
		return ret;
	}

	//2つのwaypointからなる直線と車両位置との垂線距離を計算
	double positionAndPathDistance(const geometry_msgs::msg::Point way1, const geometry_msgs::msg::Point way2, const geometry_msgs::msg::Point pos)
	{
		double x1 = way1.x, x2 = way2.x;
		double y1 = way1.y, y2 = way2.y;
		double a = y2 - y1;
		double b = x1 - x2;
		double c = - x1 * y2 + y1 * x2;

		//double x0 = current_pose_.pose.position.x, y0 = current_pose_.pose.position.y;
		double x0 = pos.x, y0 = pos.y;
		double db = sqrt(a * a + b * b);
		if(db == 0)
		{
			std::cout << "pose1とpose2が同じ" << std::endl;
			return 100;
		}
		return (a * x0 + b * y0 + c) / db;
	}

	class AutoPermissionConfig
	{
	public:
		bool forcibly_;//このフラグがtrueの場合、強制てきにpermission okにする

		bool use_ndt_tp_;//NDT精度(transform_probability)を使用するか？
		bool use_ndt_nvtl_;//NDT精度(nearest_voxel_transformation_likelihood)を使用するか？
		float ndt_tp_min_;//NDT精度(transform_probability)しきい値
		float ndt_nvtl_max_;//NDT精度(nearest_voxel_transformation_likelihood)しきい値
		float ndt_nvtl_min_;//NDT精度(nearest_voxel_transformation_likelihood)しきい値
		int ndt_tp_over_count_th_; //NDT精度(transform_probability)低下がこの回数発生したらエラー
		int ndt_nvtl_over_count_th_; //NDT精度(nearest_voxel_transformation_likelihood)低下がこの回数発生したらエラー
		int position_distance_count_th; //経路との誤差がこの回数発生したらエラー

		bool use_gnss_dev_;//gnssのdevを使用するか？
		double gnss_lat_dev_th_; //GNSS latしきい値
		double gnss_lon_dev_th_; //GNSS lonしきい値
		double gnss_alt_dev_th_; //GNSS altしきい値

		bool use_kinematic_time_diff_;//車両位置更新時間しきい値を使用するか？
		double kinematic_time_diff_th_; //車両位置更新時間しきい値

		bool use_position_and_traj_;//車両位置と経路の関係を使用するか？
		bool use_angle_and_traj_;//車両向きと経路の関係を使用するか？
		double position_and_traj_distance_th_; //車両位置と経路との距離しきい値
		double position_and_traj_angular_th_deg_; //車両位置と経路との向きしきい値

		bool use_traj_time_diff_;//経路(Trajectory)置更新時間しきい値を使用するか？
		double traj_time_diff_th_;//経路(Trajectory)置更新時間しきい値

		bool use_handle_mode_;//ハンドルがAUTOモードかを確認するか？
		bool use_pedal_mode_;//ペダルがAUTOモードかを確認するか？
	};

	class AutoPermissionStatus
	{
	public:
		float ndt_tp_; //NDT精度(transform_probability)
		float ndt_nvtl_;//NDT精度(nearest_voxel_transformation_likelihood)
		int ndt_tp_over_count_; //NDT精度(transform_probability)低下発生回数
		int ndt_nvtl_over_count_; //NDT精度(nearest_voxel_transformation_likelihood)低下発生回数
		int position_distance_count_; //経路との誤差発生回数

		geometry_msgs::msg::Vector3Stamped gnss_dev_; //GNSS標準偏差

		nav_msgs::msg::Odometry kinematic_stat_;//車両位置と車両速度
		rclcpp::Time kinematic_stat_time_;//kinematic_statの更新時間
		bool is_read_kinematic_stat_;//kinematic_statをsubscribしたか？
		double kinematic_stat_timediff_;//kinematicの更新時間の差分

		double position_and_traj_distance_; //車両位置と経路との距離
		double position_and_traj_angular_deg_; //車両位置と経路との向き
		autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_;//走行経路(Trajectory)
		rclcpp::Time traj_time_;//走行経路(Trajectory)の更新時間
		std::vector<double> traj_curvature_;//走行経路(Trajectory)の曲率
		bool is_read_traj_; //走行経路(Trajectory)をsubscribeしたか?
		double traj_timediff_;//走行経路(Trajectory)の更新時間の差分

		bool steer_auto_mode_;//ステアの自動モード
		bool pedal_auto_mode_;//ペダルの自動モード

		AutoPermissionStatus(const rclcpp::Time &time)
			: ndt_tp_(0.0)
			, ndt_nvtl_(0.0)
			, ndt_tp_over_count_(10)
			, kinematic_stat_time_(time)
			, is_read_kinematic_stat_(false)
			, kinematic_stat_timediff_(100)
			, position_and_traj_distance_(100)
			, position_and_traj_angular_deg_(180)
			, traj_time_(time)
			, is_read_traj_(false)
			, traj_timediff_(100)
			, steer_auto_mode_(false)
			, pedal_auto_mode_(false)
		{
			gnss_dev_.vector.x = gnss_dev_.vector.y = gnss_dev_.vector.z = 100;
		}
	};

	class AutoPermission : public rclcpp::Node
	{
	private:
		rclcpp::Publisher<auto_permission_msgs::msg::AutoPermission>::SharedPtr pub_auto_permission_;//現在の自動運転許可情報
		rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr pub_overlay_error_text_;//rviz表示用メッセージ

		rclcpp::Subscription<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr sub_ndt_tp_;//NDT精度
		rclcpp::Subscription<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr sub_ndt_nvtl_;//NDT精度
		rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_gnss_dev_;//GNSS標準偏差
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_stat_; //車両位置,車両速度
		rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_scenario_trajectry_;//現在の走行経路(trajectory)
		rclcpp::Subscription<autoware_auto_planning_msgs::msg::Path>::SharedPtr sub_scenario_path_;//現在の走行経路(path)
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_steer_auto_mode_;//ステアの自動モード
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pedal_auto_mode_;//ペダルの自動モード

		rclcpp::TimerBase::SharedPtr timer_;

		AutoPermissionConfig config_;
		AutoPermissionStatus status_;
		int marker_id_;

		void callbackNdtTransformProbability(const tier4_debug_msgs::msg::Float32Stamped::ConstSharedPtr stat)
		{
			status_.ndt_tp_ = stat->data;
			if(stat->data <= config_.ndt_tp_min_)
			{
				status_.ndt_tp_over_count_++;
			}
			else
			{
				status_.ndt_tp_over_count_ = 0;
			}
		}

		void callbackNdtNearestVoxelTransformationLikelihood(const tier4_debug_msgs::msg::Float32Stamped::ConstSharedPtr stat)
		{
			status_.ndt_nvtl_ = stat->data;
			if(stat->data <= config_.ndt_nvtl_min_)
			{
				status_.ndt_nvtl_over_count_++;
			}
			else
			{
				status_.ndt_nvtl_over_count_ = 0;
			}
		}

		void callbackGnssDev(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr dev)
		{
			status_.gnss_dev_ = *dev;
		}

		void callbackKinematicStat(const nav_msgs::msg::Odometry::ConstSharedPtr stat)
		{
			rclcpp::Time rosnowtime = this->now();
			//status_.kinematic_stat_timediff_ = (status_.kinematic_stat_time_ - rosnowtime).seconds();
			status_.kinematic_stat_time_ = rosnowtime;
			status_.kinematic_stat_ = *stat;
			status_.is_read_kinematic_stat_ = true;

		}

		void callbackTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory)
		{
			rclcpp::Time rosnowtime = this->now();
			//status_.traj_timediff_ = (status_.traj_time_ - rosnowtime).seconds();
			status_.traj_time_ = rosnowtime;
			status_.trajectory_ = trajectory;
			status_.is_read_traj_ = true;
			calcDistance();
			calcTrajCurvature();
		}

		void callbackSteerAutoMode(const std_msgs::msg::Bool::ConstSharedPtr mode)
		{
			status_.steer_auto_mode_ = mode->data;
		}

		void callbackPedalAutoMode(const std_msgs::msg::Bool::ConstSharedPtr mode)
		{
			status_.pedal_auto_mode_ = mode->data;
		}

		void callbackTimer()
		{
			permissionPublish();
		}

		void permissionPublish()
		{
			rclcpp::Time nowtime = this->now();

			auto_permission_msgs::msg::AutoPermission perm;// = std::make_unique<auto_permission_msgs::msg::AutoPermission>();
			perm.steer_permission = auto_permission_msgs::msg::AutoPermission::PERM_OK;
			perm.pedal_permission = auto_permission_msgs::msg::AutoPermission::PERM_OK;
			perm.use_ndt_tp = config_.use_ndt_tp_;
			perm.ndt_tp = status_.ndt_tp_;
			perm.ndt_tp_min = config_.ndt_tp_min_;
			perm.use_ndt_nvtl = config_.use_ndt_nvtl_;
			perm.ndt_nvtl = status_.ndt_nvtl_;
			perm.ndt_nvtl_max = config_.ndt_nvtl_max_;
			perm.ndt_nvtl_min = config_.ndt_nvtl_min_;
			perm.use_gnss_dev = config_.use_gnss_dev_;
			perm.gnss_lat_dev = status_.gnss_dev_.vector.x;
			perm.gnss_lat_dev_th = config_.gnss_lat_dev_th_;
			perm.gnss_lon_dev = status_.gnss_dev_.vector.y;
			perm.gnss_lon_dev_th = config_.gnss_lon_dev_th_;
			perm.gnss_alt_dev = status_.gnss_dev_.vector.z;
			perm.gnss_alt_dev_th = config_.gnss_alt_dev_th_;
			perm.gnss_yaw_dev_th = 1.0;
			perm.use_kinematic_stat = config_.use_kinematic_time_diff_;
			perm.kinematic_time_diff = status_.kinematic_stat_timediff_;
			perm.kinematic_time_diff_th = config_.kinematic_time_diff_th_;
			perm.use_position_and_traj = config_.use_position_and_traj_;
			perm.use_angle_and_traj = config_.use_angle_and_traj_;
			perm.position_and_traj_distance = status_.position_and_traj_distance_;
			perm.position_and_traj_distance_th = config_.position_and_traj_distance_th_;
			perm.position_and_traj_augular_deg = status_.position_and_traj_angular_deg_;
			perm.position_and_traj_augular_th_deg = config_.position_and_traj_angular_th_deg_;
			perm.use_trajectory_time_diff = config_.use_traj_time_diff_;
			perm.trajectory_time_diff_th = status_.traj_timediff_;
			perm.trajectory_time_diff_th = config_.traj_time_diff_th_;
			perm.pedal_mode = status_.pedal_auto_mode_;
			perm.steer_mode = status_.steer_auto_mode_;

			std::stringstream error_str;

			if(config_.use_ndt_tp_)
			{
				if(status_.ndt_tp_over_count_ > config_.ndt_tp_over_count_th_)
				{
					perm.steer_permission |= auto_permission_msgs::msg::AutoPermission::PERM_NDT_TP;
					perm.pedal_permission |= auto_permission_msgs::msg::AutoPermission::PERM_NDT_TP;
					error_str << "NDT_TP_OVER:" << std::fixed << std::setprecision(3) << status_.ndt_tp_ << " TH:" << config_.ndt_tp_min_ << '\n';
				}
			}

			if(config_.use_ndt_nvtl_)
			{
				if(status_.ndt_nvtl_over_count_ > config_.ndt_nvtl_over_count_th_)
				{
					perm.steer_permission |= auto_permission_msgs::msg::AutoPermission::PERM_NDT_NVTL;
					perm.pedal_permission |= auto_permission_msgs::msg::AutoPermission::PERM_NDT_NVTL;
					error_str << "NDT_NVTL_OVER:" << std::fixed << std::setprecision(3) << status_.ndt_nvtl_ << " MAX:" << config_.ndt_nvtl_max_ << " MIN:" << config_.ndt_nvtl_min_ << '\n';
				}
			}

			if(config_.use_gnss_dev_)
			{
				if(status_.gnss_dev_.vector.x > config_.gnss_lat_dev_th_ ||
				   status_.gnss_dev_.vector.y > config_.gnss_lon_dev_th_ ||
				   status_.gnss_dev_.vector.z > config_.gnss_alt_dev_th_)
				{
					perm.steer_permission |= auto_permission_msgs::msg::AutoPermission::PERM_GNSS_DEV;
					perm.pedal_permission |= auto_permission_msgs::msg::AutoPermission::PERM_GNSS_DEV;
					error_str << "GNSS DEV OVER:" << std::fixed << std::setprecision(3) << status_.gnss_dev_.vector.x << "," << status_.gnss_dev_.vector.y << "," << status_.gnss_dev_.vector.z << '\n';
				}
			}

			if(config_.use_kinematic_time_diff_)
			{
				status_.kinematic_stat_timediff_ = (nowtime - status_.kinematic_stat_time_).seconds();
				if(status_.kinematic_stat_timediff_ > config_.kinematic_time_diff_th_)
				{
					perm.steer_permission |= auto_permission_msgs::msg::AutoPermission::PERM_KINEMATIC_TIME_DIFF_OVER;
					perm.pedal_permission |= auto_permission_msgs::msg::AutoPermission::PERM_KINEMATIC_TIME_DIFF_OVER;
					error_str << "KINEMATIC TIME DIFF:" << std::fixed << std::setprecision(3) << status_.kinematic_stat_timediff_ << " TH:" << config_.kinematic_time_diff_th_ << '\n';
				}
			}

			if(config_.use_position_and_traj_)
			{
				if(status_.position_distance_count_ > config_.position_distance_count_th)
				{
					perm.steer_permission |= auto_permission_msgs::msg::AutoPermission::PERM_SCENARIO_TRAJECTORY_DISTANCE;
					perm.pedal_permission |= auto_permission_msgs::msg::AutoPermission::PERM_SCENARIO_TRAJECTORY_DISTANCE;
					error_str << "TRAJECTORY DISTANCE OVER:" << std::fixed << std::setprecision(3) << status_.position_and_traj_distance_ << " TH:" << config_.position_and_traj_distance_th_ << '\n';
				}
			}

			if(config_.use_angle_and_traj_)
			{
				if(status_.position_and_traj_angular_deg_ > config_.position_and_traj_angular_th_deg_)
				{
					perm.steer_permission |= auto_permission_msgs::msg::AutoPermission::PERM_SCENARIO_TRAJECTORY_ANGULAR;
					perm.pedal_permission |= auto_permission_msgs::msg::AutoPermission::PERM_SCENARIO_TRAJECTORY_ANGULAR;
					error_str << "TRAJECTORY ANGULAR OVER:" << std::fixed << std::setprecision(3) << status_.position_and_traj_angular_deg_ << " TH:" << config_.position_and_traj_angular_th_deg_ << '\n';
				}
			}

			if(config_.use_traj_time_diff_)
			{
				status_.traj_timediff_ = (nowtime - status_.traj_time_).seconds();
				if(status_.traj_timediff_ > config_.traj_time_diff_th_)
				{
					perm.steer_permission |= auto_permission_msgs::msg::AutoPermission::PERM_TRAJECTORY_TIME_DIFF_OVER;
					perm.pedal_permission |= auto_permission_msgs::msg::AutoPermission::PERM_TRAJECTORY_TIME_DIFF_OVER;
					error_str << "TRAJECTORY TIME DIFF:" << std::fixed << std::setprecision(3) << status_.traj_timediff_ << " TH:" << config_.traj_time_diff_th_ << '\n';
				}
			}

			if(config_.use_pedal_mode_)
			{
				if(!status_.pedal_auto_mode_)
				{
					perm.steer_permission |= auto_permission_msgs::msg::AutoPermission::PERM_NOT_PEDAL_MODE;
					perm.pedal_permission |= auto_permission_msgs::msg::AutoPermission::PERM_NOT_PEDAL_MODE;
					error_str << "NOT DRIVE AUTOMODE\n";
				}
			}

			if(config_.use_handle_mode_)
			{
				if(!status_.steer_auto_mode_)
				{
					perm.steer_permission |= auto_permission_msgs::msg::AutoPermission::PERM_NOT_STEER_MODE;
					perm.pedal_permission |= auto_permission_msgs::msg::AutoPermission::PERM_NOT_STEER_MODE;
					error_str << "NOT HANDLE AUTOMODE\n";
				}
			}

			perm.forcibly = config_.forcibly_;
			if(config_.forcibly_)
			{
				perm.steer_permission = auto_permission_msgs::msg::AutoPermission::PERM_OK;
				perm.pedal_permission = auto_permission_msgs::msg::AutoPermission::PERM_OK;
			}

			pub_auto_permission_->publish(perm);

			rviz_2d_overlay_msgs::msg::OverlayText overlay_text;
			overlay_text.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
			overlay_text.width = 500;
			overlay_text.height = 200;
			overlay_text.bg_color.r = 0;
			overlay_text.bg_color.g = 0.5;
			overlay_text.bg_color.b = 0;
			overlay_text.bg_color.a = 0.3;
			overlay_text.text_size = 15;
			overlay_text.fg_color.r = 1;
			overlay_text.fg_color.g = 1;
			overlay_text.fg_color.b = 1;
			overlay_text.fg_color.a = 1;
			overlay_text.text = error_str.str();
			pub_overlay_error_text_->publish(overlay_text);
		}

		//車両位置と経路との距離を計算
		void calcDistance()
		{
			if(!status_.is_read_kinematic_stat_ || !status_.is_read_traj_)//車両位置と経路がsubscribeされていない場合
				status_.position_and_traj_distance_ = 100;
			else if(status_.trajectory_->points.size() == 0)//waypointが無い場合
				status_.position_and_traj_distance_ = 100;
			else if(status_.trajectory_->points.size() == 1)//waypointが1つだけの場合
				status_.position_and_traj_distance_ = euclidDistance(status_.kinematic_stat_.pose.pose.position, status_.trajectory_->points[0].pose.position);
			else
			{
				//車両位置とwaypoit群で一番近いwaypointを探索
				double distance_min = __DBL_MAX__;
				int distance_min_index = -1;
				for(unsigned int i=0; i<status_.trajectory_->points.size(); i++)
				{
					double distance = euclidDistance(status_.kinematic_stat_.pose.pose.position, status_.trajectory_->points[i].pose.position);
					if(config_.position_and_traj_distance_th_ < distance_min)
					{
						distance_min = distance;
						distance_min_index = (int)i;
					}
				}
				if(distance_min_index == -1)
				{
					status_.position_and_traj_distance_ = 100;
					return;
				}

				//一番近いwaypointの前後のwaypointのどちらを使用するかの選択
				int ind1, ind2;
				if(distance_min_index == 0)
				{
					ind1 = distance_min_index;
					ind2 = distance_min_index+1;
				}
				else if(distance_min_index == (int)status_.trajectory_->points.size() - 1)
				{
					ind1 = distance_min_index-1;
					ind2 = distance_min_index;
				}
				else
				{
					double dis_next = euclidDistance(status_.kinematic_stat_.pose.pose.position, status_.trajectory_->points[distance_min_index+1].pose.position);
					double dis_prev = euclidDistance(status_.kinematic_stat_.pose.pose.position, status_.trajectory_->points[distance_min_index-1].pose.position);
					if(dis_next < dis_prev)
					{
						ind1 = distance_min_index;
						ind2 = distance_min_index+1;
					}
					else
					{
						ind1 = distance_min_index-1;
						ind2 = distance_min_index;
					}
				}

				//2つのwaypointからなる直線と車両位置との垂線距離を計算
				status_.position_and_traj_distance_ = positionAndPathDistance(
					status_.trajectory_->points[ind1].pose.position,
					status_.trajectory_->points[ind2].pose.position,
					status_.kinematic_stat_.pose.pose.position
				);
				if(status_.position_and_traj_distance_ > config_.position_and_traj_distance_th_)
				{
					status_.position_distance_count_++;
				}
				else
				{
					status_.position_distance_count_ = 0;
				}


				double yaw = formedAngular(status_.trajectory_->points[ind1].pose.orientation, status_.kinematic_stat_.pose.pose.orientation).z;
				status_.position_and_traj_angular_deg_ = yaw * 180.0 / M_PI;
			}
		}

		void calcTrajCurvature()
		{
			status_.traj_curvature_.clear();
			const std::shared_ptr<const autoware_auto_planning_msgs::msg::Trajectory> traj  = status_.trajectory_;
			geometry_msgs::msg::Point p1, p2, p3;
			for(size_t i=1; i<traj->points.size()-1; i++)
			{
				const size_t curr_idx = i;
				const size_t prev_idx = curr_idx - 1;
				const size_t next_idx = curr_idx + 1;
				p1.x = traj->points[prev_idx].pose.position.x;
				p2.x = traj->points[curr_idx].pose.position.x;
				p3.x = traj->points[next_idx].pose.position.x;
				p1.y = traj->points[prev_idx].pose.position.y;
				p2.y = traj->points[curr_idx].pose.position.y;
				p3.y = traj->points[next_idx].pose.position.y;
				try{
					status_.traj_curvature_.push_back(tier4_autoware_utils::calcCurvature(p1, p2, p3));
					//RCLCPP_INFO(get_logger(), "%lu , %lf", i, status_.traj_curvature_[i]);
				}
				catch(...)
				{
					status_.traj_curvature_.push_back(0);
					//RCLCPP_INFO(get_logger(), "%lu , error", i);
				}
				
			}
		}

	public:
		AutoPermission(const rclcpp::NodeOptions &node_option)
			: rclcpp::Node("auto_permission", node_option)
			, status_(this->now())
			, marker_id_(0)
		{
			config_.forcibly_ = declare_parameter<bool>("forcibly", false);
			config_.use_ndt_tp_ = declare_parameter<bool>("use_ndt_tp", true);
			config_.use_ndt_nvtl_ = declare_parameter<bool>("use_ndt_nvtl", true);
			config_.ndt_tp_min_ = declare_parameter<float>("ndt_tp_min", 0.0);
			config_.ndt_nvtl_max_ = declare_parameter<float>("ndt_nvtl_max", 0.0);
			config_.ndt_nvtl_min_ = declare_parameter<float>("ndt_nvtl_min", 0.0);
			config_.ndt_tp_over_count_th_ = declare_parameter<int>("ndt_tp_over_count_th",0);
			config_.ndt_nvtl_over_count_th_ = declare_parameter<int>("ndt_nvtl_over_count_th",0);
			config_.position_distance_count_th = declare_parameter<int>("position_distance_count_th",0);
			config_.use_gnss_dev_ = declare_parameter<bool>("use_gnss_dev", true);
			config_.gnss_lat_dev_th_ = declare_parameter<double>("gnss_lat_dev_th", 0.0);
			config_.gnss_lon_dev_th_ = declare_parameter<double>("gnss_lon_dev_th", 0.0);
			config_.gnss_alt_dev_th_ = declare_parameter<double>("gnss_alt_dev_th", 0.0);
			config_.use_kinematic_time_diff_ = declare_parameter<bool>("use_kinematic_time_diff", true);
			config_.kinematic_time_diff_th_ = declare_parameter<double>("kinematic_time_diff_th", 0.0);
			config_.use_position_and_traj_ = declare_parameter<bool>("use_position_and_path", true);
			config_.use_angle_and_traj_ = declare_parameter<bool>("use_angle_and_path", true);
			config_.position_and_traj_distance_th_ = declare_parameter<double>("position_and_path_distance_th", 0.0);
			config_.position_and_traj_angular_th_deg_ = declare_parameter<double>("position_and_path_angular_th_deg", 0.0);
			config_.use_traj_time_diff_ = declare_parameter<bool>("use_trajectory_time_diff", true);
			config_.traj_time_diff_th_ = declare_parameter<double>("trajectory_time_diff_th", 0.0);
			config_.use_handle_mode_ = declare_parameter<bool>("use_handle_mode", true);
			config_.use_pedal_mode_ = declare_parameter<bool>("use_pedal_mode", true);

			RCLCPP_INFO(this->get_logger(), "use_ndt_tp:%d", config_.forcibly_);
			RCLCPP_INFO(this->get_logger(), "use_ndt_tp:%d", config_.use_ndt_tp_);
			RCLCPP_INFO(this->get_logger(), "ndt_tp_min:%lf", config_.ndt_tp_min_);
			RCLCPP_INFO(this->get_logger(), "ndt_tp_over_count_th:%d", config_.ndt_tp_over_count_th_);
			RCLCPP_INFO(this->get_logger(), "ndt_nvtl_over_count_th:%d", config_.ndt_nvtl_over_count_th_);
			RCLCPP_INFO(this->get_logger(), "use_gnss_dev:%d", (int)config_.use_gnss_dev_);
			RCLCPP_INFO(this->get_logger(), "gnss_lat_dev_th:%lf", config_.gnss_lat_dev_th_);
			RCLCPP_INFO(this->get_logger(), "gnss_lon_dev_th:%lf", config_.gnss_lon_dev_th_);
			RCLCPP_INFO(this->get_logger(), "gnss_alt_dev_th:%lf", config_.gnss_alt_dev_th_);
			RCLCPP_INFO(this->get_logger(), "use_kinematic_time_diff:%d", (int)config_.use_kinematic_time_diff_);
			RCLCPP_INFO(this->get_logger(), "kinematic_time_diff_th:%lf", config_.kinematic_time_diff_th_);
			RCLCPP_INFO(this->get_logger(), "use_position_and_path:%d", (int)config_.use_position_and_traj_);
			RCLCPP_INFO(this->get_logger(), "use_angle_and_path:%d", (int)config_.use_angle_and_traj_);
			RCLCPP_INFO(this->get_logger(), "position_and_path_distance_th:%lf", config_.position_and_traj_distance_th_);
			RCLCPP_INFO(this->get_logger(), "position_and_path_angular_th_deg:%lf", config_.position_and_traj_angular_th_deg_);
			RCLCPP_INFO(this->get_logger(), "use_trajectory_time_diff:%d", (int)config_.use_traj_time_diff_);
			RCLCPP_INFO(this->get_logger(), "trajectory_time_diff_th:%lf", config_.traj_time_diff_th_);
			RCLCPP_INFO(this->get_logger(), "use_handle_mode:%d", (int)config_.use_handle_mode_);
			RCLCPP_INFO(this->get_logger(), "use_pedal_mode:%d", (int)config_.use_pedal_mode_);

			timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(50).period(),
				std::bind(&AutoPermission::callbackTimer, this));

			pub_auto_permission_ = create_publisher<auto_permission_msgs::msg::AutoPermission>("auto_permission", rclcpp::QoS(5));
			pub_overlay_error_text_ = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("overlay_error_text", rclcpp::QoS(5));

			sub_ndt_tp_ = create_subscription<tier4_debug_msgs::msg::Float32Stamped>("transform_probability", rclcpp::QoS(5),
				std::bind(&AutoPermission::callbackNdtTransformProbability, this, std::placeholders::_1));
			sub_ndt_nvtl_ = create_subscription<tier4_debug_msgs::msg::Float32Stamped>("nearest_voxel_transformation_likelihood", rclcpp::QoS(5),
				std::bind(&AutoPermission::callbackNdtNearestVoxelTransformationLikelihood, this, std::placeholders::_1));
			sub_gnss_dev_ = create_subscription<geometry_msgs::msg::Vector3Stamped>("gnss_dev", rclcpp::QoS(5),
				std::bind(&AutoPermission::callbackGnssDev, this, std::placeholders::_1));
			sub_kinematic_stat_ = create_subscription<nav_msgs::msg::Odometry>("kinematic_stat", rclcpp::QoS(5),
				std::bind(&AutoPermission::callbackKinematicStat, this, std::placeholders::_1));
			sub_scenario_trajectry_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("scenario_trajectry", rclcpp::QoS(5),
				std::bind(&AutoPermission::callbackTrajectory, this, std::placeholders::_1));
			/*sub_scenario_path_ = create_subscription<autoware_auto_planning_msgs::msg::Path>("scenario_path", rclcpp::QoS(5),
				std::bind(&AutoPermission::callbackPath, this, std::placeholders::_1));*/
			sub_steer_auto_mode_ = create_subscription<std_msgs::msg::Bool>("steer_auto_mode", rclcpp::QoS(5),
				std::bind(&AutoPermission::callbackSteerAutoMode, this, std::placeholders::_1));
			sub_pedal_auto_mode_ = create_subscription<std_msgs::msg::Bool>("pedal_auto_mode", rclcpp::QoS(5),
				std::bind(&AutoPermission::callbackPedalAutoMode, this, std::placeholders::_1));
		}
	};
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<auto_permission::AutoPermission> node = std::make_shared<auto_permission::AutoPermission>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
