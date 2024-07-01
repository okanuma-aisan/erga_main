#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
//#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <mobileye_msgs/msg/obstacle_data.hpp>
#include <detect_fusion_msgs/msg/mobileye_obstacle_stamp_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
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

private://ros2 subscriber
	rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr sub_lidar_detect_;//ライダーから検出した障害物情報
	rclcpp::Subscription<mobileye_msgs::msg::ObstacleData>::SharedPtr sub_mobileye_detect_;//mobileyeからの障害物情報
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_covariance_;

private://ros2 publisher
	rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr pub_fusion_detect_;//ライダーとmobileyeを統合した障害物情報
	rclcpp::Publisher<detect_fusion_msgs::msg::MobileyeObstacleStampArray>::SharedPtr pub_debug_mobileye_array_;//保持されているmobileye情報一覧
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mobileye_marker_, pub_lidar_marker_;

private://ros2 timer
	rclcpp::TimerBase::SharedPtr timer_pub_;//統合した障害物情報を一定時間ごとにpublishするタイマー
	rclcpp::TimerBase::SharedPtr timer_delete_;//mobileyeの障害物情報が一定時間更新されない場合に消去するタイマー

private://障害物情報
	autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr lidar_detects_;//ライダーからの障害物情報
	detect_fusion_msgs::msg::MobileyeObstacleStampArray mobileye_detects_;//mobileyeからの障害物情報一覧

	geometry_msgs::msg::PoseWithCovarianceStamped localization_pose_with_covariance_;

private://座標変換
	Eigen::Matrix4d transform_btom_;
	Eigen::Affine3d affine_btomo_;

private://ros2 callback
	//ライダーからの障害物情報のcallback
	void callbackLidarDetect(const autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr detects)
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

	void callbackPoseWithCovariance(const geometry_msgs::msg::PoseWithCovarianceStamped pose)
	{
		localization_pose_with_covariance_ = pose;
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

	visualization_msgs::msg::Marker::SharedPtr get_lidar_marker_ptr(const autoware_auto_perception_msgs::msg::DetectedObject &data)
	{
		auto marker_ptr = std::make_shared<Marker>();
		marker_ptr->header.frame_id = "map";
		marker_ptr->header.stamp = rclcpp::Time();
		marker_ptr->type = visualization_msgs::msg::Marker::SPHERE;
		marker_ptr->ns = std::string("sphere");
		marker_ptr->action = visualization_msgs::msg::Marker::ADD;
		marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
		marker_ptr->scale.x = 1;
		marker_ptr->scale.y = 1;
		marker_ptr->scale.z = 1;
		marker_ptr->pose.position.x = data.kinematics.pose_with_covariance.pose.position.x;
		marker_ptr->pose.position.y = data.kinematics.pose_with_covariance.pose.position.y;
		marker_ptr->pose.position.z = data.kinematics.pose_with_covariance.pose.position.z;
		marker_ptr->color.r = 255;
		marker_ptr->color.g = 0;
		marker_ptr->color.b = 0;
		marker_ptr->color.a = 1;

		return marker_ptr;
	}

	visualization_msgs::msg::Marker::SharedPtr get_mobileye_marker_ptr(
		detect_fusion_msgs::msg::MobileyeObstacleStamp data
	)
	{
		auto marker_ptr = std::make_shared<Marker>();
		marker_ptr->header.frame_id = "map";
		marker_ptr->header.stamp = rclcpp::Time();
		marker_ptr->type = visualization_msgs::msg::Marker::SPHERE;
		marker_ptr->ns = std::string("sphere");
		marker_ptr->action = visualization_msgs::msg::Marker::ADD;
		marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
		marker_ptr->scale.x = 1.0;
		marker_ptr->scale.y = 1.0;
		marker_ptr->scale.z = 1.0;
		marker_ptr->pose.position.x = data.mobileye_obstacle.obstacle_position_x;
		marker_ptr->pose.position.y = data.mobileye_obstacle.obstacle_position_y;
		marker_ptr->pose.position.z = localization_pose_with_covariance_.pose.pose.position.z + 1.0;
		marker_ptr->color.r = 255;
		marker_ptr->color.g = 255;
		marker_ptr->color.b = 0;
		marker_ptr->color.a = 1;

		return marker_ptr;
	}


private://その他
	//統合した障害物情報をpublish
	void detectPublish()
	{
		if(lidar_detects_->objects.size() != 0 || mobileye_detects_.array.size() != 0)
		{
			autoware_auto_perception_msgs::msg::DetectedObjects detects(*lidar_detects_);
			for(const auto & object : lidar_detects_->objects)
			{
				pub_lidar_marker_->publish(*get_lidar_marker_ptr(object));
			}

			//mobileyeのデータをlidarに統合する
			for(const auto & object : mobileye_detects_.array)
			{
				autoware_auto_perception_msgs::msg::DetectedObject lidar_detects_object;

				autoware_auto_perception_msgs::msg::ObjectClassification classification;
				classification.probability = 1.0;
				if(object.mobileye_obstacle.obstacle_type == 0)//vehicle
					classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
				else if(object.mobileye_obstacle.obstacle_type == 1)//Truc
					classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK;
				else if(object.mobileye_obstacle.obstacle_type == 2)//Bike
					classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
				else if(object.mobileye_obstacle.obstacle_type == 3)//ped
					classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
				else if(object.mobileye_obstacle.obstacle_type == 4)//bicycle
					classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE;
				else
					classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
				lidar_detects_object.classification.push_back(classification);

				lidar_detects_object.kinematics.pose_with_covariance.pose.position.x = object.mobileye_obstacle.obstacle_position_x;
				lidar_detects_object.kinematics.pose_with_covariance.pose.position.y = object.mobileye_obstacle.obstacle_position_y;
				lidar_detects_object.kinematics.pose_with_covariance.pose.position.z = localization_pose_with_covariance_.pose.pose.position.z + 1.0;
				tf2::Quaternion qua;
				qua.setRPY(0.0, 0.0, object.mobileye_obstacle.obstacle_angle*M_PI/180.0);
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
				lidar_detects_object.shape.dimensions.x = object.mobileye_obstacle.obstacle_length;
				lidar_detects_object.shape.dimensions.y = object.mobileye_obstacle.obstacle_width;

				detects.objects.push_back(lidar_detects_object);

				pub_mobileye_marker_->publish(*get_mobileye_marker_ptr(object));
			}
			pub_fusion_detect_->publish(detects);
		}
	}

public:
	DetectFusion(const rclcpp::NodeOptions &node_options)
		: rclcpp::Node("detect_fusion", node_options)
		, publish_hz_(declare_parameter<double>("publish_hz", 10.0))
		, mobileye_delete_time_th_(declare_parameter<double>("mobileye_delete_time_th", 0.5))
		, lidar_detects_(std::make_shared<autoware_auto_perception_msgs::msg::PredictedObjects>())
	{
		RCLCPP_INFO(get_logger(), "publish_hz,%lf", publish_hz_);

		double mox, moy, moz, moyaw, mopitch, moroll;
		mox = 7.6;  moy = 0.0; moz = 1.5;
		moyaw = 0.0;  mopitch = 0.0;  moroll = 0.0;
		Eigen::Translation3d tl_btom(mox, moy, moz);                 // tl: translation
		Eigen::AngleAxisd rot_x_btom(moroll, Eigen::Vector3d::UnitX());  // rot: rotation
		Eigen::AngleAxisd rot_y_btom(mopitch, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd rot_z_btom(moyaw, Eigen::Vector3d::UnitZ());
		transform_btom_ = (tl_btom * rot_z_btom * rot_y_btom * rot_x_btom).matrix();
		affine_btomo_ = (tl_btom * rot_z_btom * rot_y_btom * rot_x_btom);

		pub_fusion_detect_ = create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>("out_fusion_detect", rclcpp::QoS(1));
		pub_debug_mobileye_array_ = create_publisher<detect_fusion_msgs::msg::MobileyeObstacleStampArray>("debug/mobileye_array", rclcpp::QoS(1));
		pub_mobileye_marker_ = create_publisher<visualization_msgs::msg::Marker>("mobileye_obstacle_marker", rclcpp::QoS(1));
		pub_lidar_marker_ = create_publisher<visualization_msgs::msg::Marker>("lidar_obstacle_marker", rclcpp::QoS(1));

		sub_lidar_detect_ = create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>("in_lidar_detect", rclcpp::QoS(1),
			std::bind(&DetectFusion::callbackLidarDetect, this, std::placeholders::_1));
		//障害物方法の統合で処理時間がかかることを考慮して、mobileye情報のsubscribe履歴を多めにする	
		sub_mobileye_detect_ = create_subscription<mobileye_msgs::msg::ObstacleData>("in_mobileye_detect", rclcpp::QoS(30),
			std::bind(&DetectFusion::callbackMobileyeDetect, this, std::placeholders::_1));
		sub_pose_with_covariance_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("in_localizer_pose", rclcpp::QoS(1),
			std::bind(&DetectFusion::callbackPoseWithCovariance, this, std::placeholders::_1));

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