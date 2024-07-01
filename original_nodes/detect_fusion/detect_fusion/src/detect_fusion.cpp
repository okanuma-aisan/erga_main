#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
//#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <mobileye_msgs/msg/obstacle_data.hpp>
#include <detect_fusion_msgs/msg/mobileye_obstacle_stamp_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2/utils.h>

namespace
{
using Marker = visualization_msgs::msg::Marker;

}

class DetectFusion : public rclcpp::Node
{
	
private://parameters
	const double publish_hz_;//統合した障害情報をpublishする周期
	const double mobileye_delete_time_th_;//古いmobileye情報を消去する時間
	const bool debug_marker_;//!< 障害物マーカーの表示有無

private://ros2 subscriber
	rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr sub_lidar_detect_;//ライダーから検出した障害物情報
	rclcpp::Subscription<mobileye_msgs::msg::ObstacleData>::SharedPtr sub_mobileye_detect_;//mobileyeからの障害物情報
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_stat_;//現在の車両位置と速度

private://ros2 publisher
	rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr pub_fusion_detect_;//ライダーとmobileyeを統合した障害物情報
	rclcpp::Publisher<detect_fusion_msgs::msg::MobileyeObstacleStampArray>::SharedPtr pub_debug_mobileye_array_;//保持されているmobileye情報一覧
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_lidar_marker_array_;//lidar detecterのマーカー
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mobileye_marker_array_;//mobileye detecterのマーカー

private://ros2 timer
	rclcpp::TimerBase::SharedPtr timer_pub_;//統合した障害物情報を一定時間ごとにpublishするタイマー
	rclcpp::TimerBase::SharedPtr timer_delete_;//mobileyeの障害物情報が一定時間更新されない場合に消去するタイマー

private://障害物情報
	autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr lidar_detects_;//ライダーからの障害物情報
	detect_fusion_msgs::msg::MobileyeObstacleStampArray mobileye_detects_;//mobileyeからの障害物情報一覧

	nav_msgs::msg::Odometry kinematic_stat_;//現在の車両位置と速度

private://座標変換
	Eigen::Matrix4d transform_btom_;
	Eigen::Affine3d affine_btomo_;

private://車両情報
	bool use_lidar_detect_;//LIDARの障害物情報をpublishに加える
	bool use_mobileye_detect_;//MOBILEYEの障害物情報をpublishに加える
	double base_to_front_;//base_linkからフロントまでの距離
	double mob_vehicle_length_;//mobileyeで検知した車両に設定する車両長さ(普通車)
	double mob_vehicle_height_;//mobileyeで検知した車両に設定する車両高さ(普通車)
	double mob_truck_length_;//mobileyeで検知した車両に設定する車両長さ(トラック)
	double mob_truck_height_;//mobileyeで検知した車両に設定する車両高さ(トラック)
	double mob_motorcycle_length_;//mobileyeで検知した車両に設定する車両長さ(バイク)
	double mob_motorcycle_height_;//mobileyeで検知した車両に設定する車両高さ(バイク)
	double mob_pedestrian_height_;//mobileyeで検知した車両に設定する車両高さ(人間)
	double mob_bicycle_length_;//mobileyeで検知した車両に設定する車両長さ(自転車)
	double mob_bicycle_height_;//mobileyeで検知した車両に設定する車両高さ(自転車)
	double mob_unknown_length_;//mobileyeで検知した車両に設定する車両長さ(謎)
	double mob_unknown_height_;//mobileyeで検知した車両に設定する車両高さ(謎)

private://ros2 callback
	//ライダーからの障害物情報のcallback
	void callbackLidarDetect(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr detects)
	{
		lidar_detects_ = detects;
		//ライダーからの障害物情報は、その時間の全データが一度にsubscribeされる。
		//なのでライダーから障害物情報をsubした段階で、mobileyeの障害物情報を統合してpublishする
		detectPublish();
		timer_pub_.reset();//過剰なpublishをしないように、publish用timerの実行カウントを0にする
	}

	//mobileyeからの障害物情報のcallback
	void callbackMobileyeDetect(const mobileye_msgs::msg::ObstacleData::ConstSharedPtr detect)
	{
		//ここでmobileye_detects_に障害物情報を追加する
		//同一IDはかぶらないように注意
		size_t count;
		for(count=0; count<mobileye_detects_.array.size(); count++)
		{
			const mobileye_msgs::msg::ObstacleData &obs = mobileye_detects_.array[count].mobileye_obstacle;
			if(obs.obstacle_id == detect->obstacle_id)//同一IDがある場合は、情報を更新
			{
				mobileye_detects_.array.erase(mobileye_detects_.array.begin()+count, mobileye_detects_.array.begin()+count+1);
				detect_fusion_msgs::msg::MobileyeObstacleStamp data;
				data.stamp = this->now();
				data.mobileye_obstacle = *detect;
				//車体の向き等を考慮する
				//data.mobileye_obstacle = get_mobileye_position(*detect);
				mobileye_detects_.array.push_back(data);	
				break;
			}
		}
		if(count == mobileye_detects_.array.size())//IDが重複していない場合は、そのままmobileye_detectsに追加
		{
			detect_fusion_msgs::msg::MobileyeObstacleStamp data;
			data.stamp = this->now();
			data.mobileye_obstacle = *detect;
			//車体の向き等を考慮する
			//data.mobileye_obstacle = get_mobileye_position(*detect);
			mobileye_detects_.array.push_back(data);
		}

		//mobileyeからの障害物情報は１つの障害物情報のみ。
		//なので、メンバ変数に車両ID毎の情報をstd::vector等で保存して、タイマーでpublishするのがいいと思います。
		//ライダーの障害物情報がsubscribeされた場合は、ライダー側のcallbackで統合情報をpublishするといいでしょう。
	}

	//統合した障害物情報をpublishするタイマー
	void callbackPubTimer()
	{
		detectPublish();
	}

	//mobileyeの障害物情報が一定時間更新されない場合に消去するタイマー
	void callbackDeleteTimer()
	{
		rclcpp::Time nowtime = this->now();
		for(int i=(int)mobileye_detects_.array.size()-1; i>=0; i--)
		{
			double timediff = (nowtime - mobileye_detects_.array[i].stamp).seconds();
			if(timediff > mobileye_delete_time_th_)
			{
				mobileye_detects_.array.erase(mobileye_detects_.array.begin()+i, mobileye_detects_.array.begin()+i+1);
			}
		}

		pub_debug_mobileye_array_->publish(mobileye_detects_);
	}

	void callbackKinematicStat(const nav_msgs::msg::Odometry::ConstSharedPtr stat)
	{
		kinematic_stat_ = *stat;
	}

	/*mobileye_msgs::msg::ObstacleData get_mobileye_position(const mobileye_msgs::msg::ObstacleData &detect)
	{
		Eigen::Vector3d vec_obs(detect.obstacle_position_x, detect.obstacle_position_y, 0);
		Eigen::Vector3d vec_base_to_obs = affine_btomo_ * vec_obs;

		Eigen::Translation3d tl_cur(localization_pose_with_covariance_.pose.pose.position.x, localization_pose_with_covariance_.pose.pose.position.y,
			localization_pose_with_covariance_.pose.pose.position.z);
		Eigen::Quaterniond qua_cur(localization_pose_with_covariance_.pose.pose.orientation.w, localization_pose_with_covariance_.pose.pose.orientation.x,
			                           localization_pose_with_covariance_.pose.pose.orientation.y, localization_pose_with_covariance_.pose.pose.orientation.z);
		Eigen::Affine3d affine_cur = tl_cur * qua_cur;
		Eigen::Vector3d map_obs_pose = affine_cur * vec_base_to_obs;

		mobileye_msgs::msg::ObstacleData object = detect;
		object.obstacle_position_x = map_obs_pose[0];
		object.obstacle_position_y = map_obs_pose[1];
		return object;
	}*/

	void publish_lidar_markers()
	{
		visualization_msgs::msg::MarkerArray marker_array;
		//for(const auto & object : lidar_detects_->objects)
		for(size_t i=0; i<lidar_detects_->objects.size(); i++)
		{
			const auto &object = lidar_detects_->objects[i];
			visualization_msgs::msg::Marker marker;
			marker.id = (int32_t)i;
			marker.header.frame_id = "base_link";
			marker.header.stamp = rclcpp::Time();
			marker.type = visualization_msgs::msg::Marker::SPHERE;
			marker.ns = std::string("sphere");
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.lifetime = rclcpp::Duration::from_seconds(0.2);
			marker.scale.x = 1;
			marker.scale.y = 1;
			marker.scale.z = 1;
			marker.pose.position.x = object.kinematics.pose_with_covariance.pose.position.x;
			marker.pose.position.y = object.kinematics.pose_with_covariance.pose.position.y;
			marker.pose.position.z = object.kinematics.pose_with_covariance.pose.position.z;
			marker.color.r = 255;
			marker.color.g = 0;
			marker.color.b = 0;
			marker.color.a = 1;
			marker_array.markers.push_back(marker);
		}

		pub_lidar_marker_array_->publish(marker_array);
	}

	void publish_mobileye_markers()
	{
		visualization_msgs::msg::MarkerArray marker_array;
		for(size_t i=0; i<mobileye_detects_.array.size(); i++)
		{
			const auto &object = mobileye_detects_.array[i];
			double length, height;
			uint8_t dummy;
			getVehicleSize(object, dummy, length, height);

			visualization_msgs::msg::Marker marker;
			marker.id = (int32_t)(i+1000);
			marker.header.frame_id = "base_link";
			marker.header.stamp = rclcpp::Time();
			marker.type = visualization_msgs::msg::Marker::SPHERE;
			marker.ns = std::string("sphere");
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.lifetime = rclcpp::Duration::from_seconds(0.2);
			marker.scale.x = 1.0;
			marker.scale.y = 1.0;
			marker.scale.z = 1.0;
			marker.pose.position.x = object.mobileye_obstacle.obstacle_position_x + base_to_front_ + length / 2.0;
			marker.pose.position.y = object.mobileye_obstacle.obstacle_position_y;
			marker.pose.position.z = 1.0;//localization_pose_with_covariance_.pose.pose.position.z + 1.0;
			marker.color.r = 255;
			marker.color.g = 255;
			marker.color.b = 0;
			marker.color.a = 1;
			marker_array.markers.push_back(marker);
		}

		pub_mobileye_marker_array_->publish(marker_array);
	}


private://その他
	void getVehicleSize(const detect_fusion_msgs::msg::MobileyeObstacleStamp object, uint8_t &label,
		double &length, double &height)
	{
		if(object.mobileye_obstacle.obstacle_type == 0)//vehicle
		{
			label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
			length = mob_vehicle_length_;
			height = mob_vehicle_height_;
		}
		else if(object.mobileye_obstacle.obstacle_type == 1)//Truc
		{
			label = autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK;
			length = mob_truck_length_;
			height = mob_truck_height_;
		}
		else if(object.mobileye_obstacle.obstacle_type == 2)//Bike
		{
			label = autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
			length = mob_motorcycle_length_;
			height = mob_motorcycle_height_;
		}
		else if(object.mobileye_obstacle.obstacle_type == 3)//pedestrian
		{
			label = autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
			length = object.mobileye_obstacle.obstacle_width;
			height = mob_pedestrian_height_;
		}
		else if(object.mobileye_obstacle.obstacle_type == 4)//bicycle
		{
			label = autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE;
			length = mob_bicycle_length_;
			height = mob_bicycle_height_;
		}
		else
		{
			label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
			length = mob_unknown_length_;
			height = mob_unknown_height_;
		}
	}

	//統合した障害物情報をpublish
	void detectPublish()
	{
		if(lidar_detects_->objects.size() != 0 || mobileye_detects_.array.size() != 0)
		{
			if(debug_marker_)
			{
				publish_lidar_markers();
				publish_mobileye_markers();
			}

			//mobileyeのデータをlidarに統合する
			autoware_auto_perception_msgs::msg::DetectedObjects detects = *lidar_detects_;
			if(use_lidar_detect_ == false) detects.objects.clear();

			if(use_mobileye_detect_ == true)
			{
				for(const auto & object : mobileye_detects_.array)
				{
					autoware_auto_perception_msgs::msg::DetectedObject lidar_detects_object;

					double length;
					double height;
					autoware_auto_perception_msgs::msg::ObjectClassification classification;
					classification.probability = 1.0;
					getVehicleSize(object, classification.label, length, height);

					if(object.mobileye_obstacle.obstacle_type == 0)//vehicle
					{
						classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
						length = mob_vehicle_length_;
						height = mob_vehicle_height_;
					}
					else if(object.mobileye_obstacle.obstacle_type == 1)//Truc
					{
						classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK;
						length = mob_truck_length_;
						height = mob_truck_height_;
					}
					else if(object.mobileye_obstacle.obstacle_type == 2)//Bike
					{
						classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
						length = mob_motorcycle_length_;
						height = mob_motorcycle_height_;
					}
					else if(object.mobileye_obstacle.obstacle_type == 3)//pedestrian
					{
						classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
						length = object.mobileye_obstacle.obstacle_width;
						height = mob_pedestrian_height_;
					}
					else if(object.mobileye_obstacle.obstacle_type == 4)//bicycle
					{
						classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE;
						length = mob_bicycle_length_;
						height = mob_bicycle_height_;
					}
					else
					{
						classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
						length = mob_unknown_length_;
						height = mob_unknown_height_;
					}

					lidar_detects_object.classification.push_back(classification);

					lidar_detects_object.kinematics.pose_with_covariance.pose.position.x = object.mobileye_obstacle.obstacle_position_x + base_to_front_ + length / 2.0;
					lidar_detects_object.kinematics.pose_with_covariance.pose.position.y = object.mobileye_obstacle.obstacle_position_y;
					lidar_detects_object.kinematics.pose_with_covariance.pose.position.z = 1.0;//localization_pose_with_covariance_.pose.pose.position.z + 1.0;
					tf2::Quaternion qua;
					qua.setRPY(0.0, 0.0, 0.0);//object.mobileye_obstacle.obstacle_angle*M_PI/180.0);
					lidar_detects_object.kinematics.pose_with_covariance.pose.orientation.x = qua.x();
					lidar_detects_object.kinematics.pose_with_covariance.pose.orientation.y = qua.y();
					lidar_detects_object.kinematics.pose_with_covariance.pose.orientation.z = qua.z();
					lidar_detects_object.kinematics.pose_with_covariance.pose.orientation.w = qua.w();
					lidar_detects_object.kinematics.has_position_covariance = false;

					lidar_detects_object.kinematics.twist_with_covariance.twist.linear.x = object.mobileye_obstacle.obstacle_relative_velocity_x;
					lidar_detects_object.kinematics.has_twist = false;
					lidar_detects_object.kinematics.has_twist_covariance = false;

					lidar_detects_object.kinematics.orientation_availability = 0;

					lidar_detects_object.shape.type = 0;
					lidar_detects_object.shape.dimensions.x = length;
					lidar_detects_object.shape.dimensions.y = object.mobileye_obstacle.obstacle_width;
					lidar_detects_object.shape.dimensions.z = height;

					lidar_detects_object.detection_device = autoware_auto_perception_msgs::msg::DetectedObject::DEVICE_MOBILEYE;
					detects.objects.push_back(lidar_detects_object);
				}
			}

			pub_fusion_detect_->publish(detects);
		}
	}

public:
	DetectFusion(const rclcpp::NodeOptions &node_options)
		: rclcpp::Node("detect_fusion", node_options)
		, publish_hz_(declare_parameter<double>("publish_hz", 10.0))
		, mobileye_delete_time_th_(declare_parameter<double>("mobileye_delete_time_th", 0.5))
		, debug_marker_(declare_parameter<bool>("debug_marker", false))
		, lidar_detects_(std::make_shared<autoware_auto_perception_msgs::msg::DetectedObjects>())
		, use_lidar_detect_(declare_parameter<bool>("use_lidar_detect", true))
		, use_mobileye_detect_(declare_parameter<bool>("use_mobileye_detect", true))
		, mob_vehicle_length_(declare_parameter<double>("mob_vehicle_length", 5.0))
		, mob_vehicle_height_(declare_parameter<double>("mob_vehicle_height", 1.7))
		, mob_truck_length_(declare_parameter<double>("mob_truck_length", 10.0))
		, mob_truck_height_(declare_parameter<double>("mob_truck_height", 3.0))
		, mob_motorcycle_length_(declare_parameter<double>("mob_motorcycle", 2.2))
		, mob_motorcycle_height_(declare_parameter<double>("mob_motorcycle_height", 1.8))
		, mob_pedestrian_height_(declare_parameter<double>("mob_pedestrian_height", 1.7))
		, mob_bicycle_length_(declare_parameter<double>("mob_bicycle_length", 5.0))
		, mob_bicycle_height_(declare_parameter<double>("mob_bicycle_height", 1.8))
		, mob_unknown_length_(declare_parameter<double>("mob_unknown_length", 1.0))
		, mob_unknown_height_(declare_parameter<double>("mob_unknown_height", 1.0))
	{
		double wheel_base = declare_parameter<double>("wheel_base", 2.0);
		double front_overhang = declare_parameter<double>("front_overhang", 1.0);
		base_to_front_ = wheel_base + front_overhang;

		RCLCPP_INFO(get_logger(), "publish_hz,%lf", publish_hz_);
		RCLCPP_INFO(get_logger(), "wheel_base,%lf", wheel_base);
		RCLCPP_INFO(get_logger(), "front_overhang,%lf", front_overhang);

		double mox, moy, moz, moyaw, mopitch, moroll;
		mox = 7.6;  moy = 0.0; moz = 1.5;
		moyaw = 0.0;  mopitch = 0.0;  moroll = 0.0;
		Eigen::Translation3d tl_btom(mox, moy, moz);                 // tl: translation
		Eigen::AngleAxisd rot_x_btom(moroll, Eigen::Vector3d::UnitX());  // rot: rotation
		Eigen::AngleAxisd rot_y_btom(mopitch, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd rot_z_btom(moyaw, Eigen::Vector3d::UnitZ());
		transform_btom_ = (tl_btom * rot_z_btom * rot_y_btom * rot_x_btom).matrix();
		affine_btomo_ = (tl_btom * rot_z_btom * rot_y_btom * rot_x_btom);

		pub_fusion_detect_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>("out_fusion_detect", rclcpp::QoS(1));
		pub_debug_mobileye_array_ = create_publisher<detect_fusion_msgs::msg::MobileyeObstacleStampArray>("debug/mobileye_array", rclcpp::QoS(1));
		pub_lidar_marker_array_ = create_publisher<visualization_msgs::msg::MarkerArray>("lidar_obstacle_markers", rclcpp::QoS(1));
		pub_mobileye_marker_array_ = create_publisher<visualization_msgs::msg::MarkerArray>("mobileye_obstacle_markers", rclcpp::QoS(1));

		sub_lidar_detect_ = create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>("in_lidar_detect", rclcpp::QoS(1),
			std::bind(&DetectFusion::callbackLidarDetect, this, std::placeholders::_1));
		//障害物方法の統合で処理時間がかかることを考慮して、mobileye情報のsubscribe履歴を多めにする	
		sub_mobileye_detect_ = create_subscription<mobileye_msgs::msg::ObstacleData>("in_mobileye_detect", rclcpp::QoS(30),
			std::bind(&DetectFusion::callbackMobileyeDetect, this, std::placeholders::_1));
		sub_kinematic_stat_ = create_subscription<nav_msgs::msg::Odometry>("in_kinematic_state", rclcpp::QoS(1),
			std::bind(&DetectFusion::callbackKinematicStat, this, std::placeholders::_1));

		timer_pub_ = rclcpp::create_timer(
			this, get_clock(), rclcpp::Rate(publish_hz_).period(), std::bind(&DetectFusion::callbackPubTimer, this));
		timer_delete_ = rclcpp::create_timer(
			this, get_clock(), rclcpp::Rate(publish_hz_).period(), std::bind(&DetectFusion::callbackDeleteTimer, this));
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<DetectFusion> node = std::make_shared<DetectFusion>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
