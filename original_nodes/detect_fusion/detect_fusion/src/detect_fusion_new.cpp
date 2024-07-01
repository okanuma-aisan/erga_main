#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <mobileye_msgs/msg/obstacle_data.hpp>
#include <detect_fusion_msgs/msg/mobileye_obstacle_stamp_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <eigen3/Eigen/Core>
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
	rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr sub_lidar_detect_;//ライダーから検出した障害物情報
	rclcpp::Subscription<mobileye_msgs::msg::ObstacleData>::SharedPtr sub_mobileye_detect_;//mobileyeからの障害物情報
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_covariance_;//現在の車両位置

private://ros2 publisher
	rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr pub_fusion_detect_;//ライダーとmobileyeを統合した障害物情報
	rclcpp::Publisher<detect_fusion_msgs::msg::MobileyeObstacleStampArray>::SharedPtr pub_debug_mobileye_array_;//保持されているmobileye情報一覧
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mobileye_marker_;//mobileyteオブジェクトのマーカー
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_lidar_marker_;//lidar detectのマーカ

private://ros2 timer
	rclcpp::TimerBase::SharedPtr timer_pub_;//統合した障害物情報を一定時間ごとにpublishするタイマー
	rclcpp::TimerBase::SharedPtr timer_delete_;//mobileyeの障害物情報が一定時間更新されない場合に消去するタイマー

private://障害物情報
	autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr lidar_detects_;//ライダーからの障害物情報
	detect_fusion_msgs::msg::MobileyeObstacleStampArray mobileye_detects_;//mobileyeからの障害物情報一覧

	geometry_msgs::msg::PoseWithCovarianceStamped localization_pose_with_covariance;
private://ros2 callback
	//ライダーからの障害物情報のcallback
	void callbackLidarDetect(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr detects)
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
				data.mobileye_obstacle = get_mobileye_position(*detect);
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
			data.mobileye_obstacle = get_mobileye_position(*detect);
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
		localization_pose_with_covariance = pose;
	}

	mobileye_msgs::msg::ObstacleData get_mobileye_position(mobileye_msgs::msg::ObstacleData detect)
	{
		mobileye_msgs::msg::ObstacleData object = detect;
		//baselinkとのずれ、orientationを計算する
		
		Eigen::Vector3d vec_obs(object.obstacle_position_x + 7.6,object.obstacle_position_y,0);
		double length = sqrt(pow(object.obstacle_position_x + 7.6,2) + pow(object.obstacle_position_y, 2));
		auto yaw = tf2::getYaw(localization_pose_with_covariance.pose.pose.orientation);
		double len_x, len_y;
		len_x = length * std::cos(yaw);
		len_y = length * std::sin(yaw);

		object.obstacle_position_x = localization_pose_with_covariance.pose.pose.position.x +len_x;
		object.obstacle_position_y = localization_pose_with_covariance.pose.pose.position.y +len_y;


		return object;
	}

	visualization_msgs::msg::Marker::SharedPtr get_lidar_marker_ptr(
	autoware_auto_perception_msgs::msg::PredictedObject data
	)
	{
		auto marker_ptr = std::make_shared<Marker>();
		marker_ptr->header.frame_id = "map";
		marker_ptr->header.stamp = rclcpp::Time();
		marker_ptr->type = visualization_msgs::msg::Marker::SPHERE;
		marker_ptr->ns = std::string("sphere");
		marker_ptr->action = visualization_msgs::msg::Marker::ADD;
		marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
		marker_ptr->scale.x = 0.5;
		marker_ptr->scale.y = 0.5;
		marker_ptr->scale.z = 0.5;
		marker_ptr->pose.position.x = data.kinematics.initial_pose_with_covariance.pose.position.x;
		marker_ptr->pose.position.y = data.kinematics.initial_pose_with_covariance.pose.position.y;
		marker_ptr->pose.position.z = data.kinematics.initial_pose_with_covariance.pose.position.z;
		marker_ptr->color.r = 255;
		marker_ptr->color.g = 0;
		marker_ptr->color.b = 0;
		marker_ptr->color.a = 1;

		return marker_ptr;
	}

	visualization_msgs::msg::MarkerArray::SharedPtr get_mobileye_marker_ptr(
		detect_fusion_msgs::msg::MobileyeObstacleStamp data
	)
	{
		visualization_msgs::msg::MarkerArray::SharedPtr marker_array;
		visualization_msgs::msg::Marker marker_ptr;
		visualization_msgs::msg::Marker marker_type;
		
		// auto marker_ptr = std::make_shared<Marker>();
		marker_ptr.header.frame_id = "map";
		marker_ptr.header.stamp = rclcpp::Time();
		marker_ptr.type = visualization_msgs::msg::Marker::SPHERE;
		marker_ptr.ns = std::string("sphere");
		marker_ptr.action = visualization_msgs::msg::Marker::ADD;
		marker_ptr.lifetime = rclcpp::Duration::from_seconds(0.2);
		marker_ptr.scale.x = 0.5;
		marker_ptr.scale.y = 0.5;
		marker_ptr.scale.z = 0.5;
		marker_ptr.pose.position.x = data.mobileye_obstacle.obstacle_position_x;
		marker_ptr.pose.position.y = data.mobileye_obstacle.obstacle_position_y;
		marker_ptr.pose.position.z = localization_pose_with_covariance.pose.pose.position.z + 1.0;
		marker_ptr.color.r = 255;
		marker_ptr.color.g = 255;
		marker_ptr.color.b = 0;
		marker_ptr.color.a = 1;

		marker_type.header.frame_id = "map";
		marker_type.header.stamp = rclcpp::Time();
		marker_type.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
		marker_type.ns = std::string("text");
		marker_type.action = visualization_msgs::msg::Marker::ADD;
		marker_type.lifetime = rclcpp::Duration::from_seconds(0.2);
		marker_type.scale.x = 1.0;
		marker_type.scale.y = 1.0;
		marker_type.scale.z = 1.0;
		marker_type.pose.position.x = data.mobileye_obstacle.obstacle_position_x;
		marker_type.pose.position.y = data.mobileye_obstacle.obstacle_position_y;
		marker_type.pose.position.z = localization_pose_with_covariance.pose.pose.position.z + 1.0;
		marker_type.color.r = 1;
		marker_type.color.g = 1;
		marker_type.color.b = 1;
		marker_type.color.a = 1;
		if(data.mobileye_obstacle.obstacle_type == 0)//vehicle
		{
			marker_type.text = "vehicle";
		}
		if(data.mobileye_obstacle.obstacle_type == 1)//Truck
		{
			marker_type.text = "Truck";
		}
		if(data.mobileye_obstacle.obstacle_type == 2)//Bike
		{
			marker_type.text = "Bike";
		}
		if(data.mobileye_obstacle.obstacle_type == 3)//Ped
		{
			marker_type.text = "Ped";
		}
		if(data.mobileye_obstacle.obstacle_type == 4)//Bicycle
		{
			marker_type.text = "Bicycle";
		}

		marker_array->markers.push_back(marker_ptr);
		marker_array->markers.push_back(marker_type);
		return marker_array;
	}


private://その他
	//統合した障害物情報をpublish
	void detectPublish()
	{
		if(lidar_detects_->objects.size() != 0 || mobileye_detects_.array.size() != 0)
		{
			autoware_auto_perception_msgs::msg::PredictedObjects detects(*lidar_detects_);
			for(const auto & object : lidar_detects_->objects)
			{
				pub_lidar_marker_->publish(*get_lidar_marker_ptr(object));
			}

			//mobileyeのデータをlidarに統合する
			/*for(const auto & object : mobileye_detects_.array)
			{
				autoware_auto_perception_msgs::msg::PredictedObject lidar_detects_object;

				if(object.mobileye_obstacle.obstacle_type == 0)//vehicle
				{
					lidar_detects_object.classification = static_cast<std::vector<autoware_auto_perception_msgs::msg::ObjectClassification_<std::allocator<void>>, 
					std::allocator<autoware_auto_perception_msgs::msg::ObjectClassification_<std::allocator<void>>>>>(1);
				}
				else if(object.mobileye_obstacle.obstacle_type == 1)//Truck
				{
					lidar_detects_object.classification = static_cast<std::vector<autoware_auto_perception_msgs::msg::ObjectClassification_<std::allocator<void>>, 
					std::allocator<autoware_auto_perception_msgs::msg::ObjectClassification_<std::allocator<void>>>>>(2);
				}
				else if(object.mobileye_obstacle.obstacle_type == 2)//Bike
				{
					lidar_detects_object.classification = static_cast<std::vector<autoware_auto_perception_msgs::msg::ObjectClassification_<std::allocator<void>>, 
					std::allocator<autoware_auto_perception_msgs::msg::ObjectClassification_<std::allocator<void>>>>>(5);
				}
				else if(object.mobileye_obstacle.obstacle_type == 3)//Ped
				{
					lidar_detects_object.classification = static_cast<std::vector<autoware_auto_perception_msgs::msg::ObjectClassification_<std::allocator<void>>, 
					std::allocator<autoware_auto_perception_msgs::msg::ObjectClassification_<std::allocator<void>>>>>(7);
				}
				else if(object.mobileye_obstacle.obstacle_type == 4)//Bicycle
				{
					lidar_detects_object.classification = static_cast<std::vector<autoware_auto_perception_msgs::msg::ObjectClassification_<std::allocator<void>>, 
					std::allocator<autoware_auto_perception_msgs::msg::ObjectClassification_<std::allocator<void>>>>>(6);
				}

				lidar_detects_object.kinematics.initial_pose_with_covariance.pose.position.x = object.mobileye_obstacle.obstacle_position_x;
				lidar_detects_object.kinematics.initial_pose_with_covariance.pose.position.y = object.mobileye_obstacle.obstacle_position_y;
				lidar_detects_object.kinematics.initial_pose_with_covariance.pose.position.z = localization_pose_with_covariance.pose.pose.position.z + 1.0;

				tf2::Quaternion qua;
				qua.setRPY(0.0, 0.0, object.mobileye_obstacle.obstacle_angle*M_PI/180.0);
				lidar_detects_object.kinematics.initial_pose_with_covariance.pose.orientation.x = qua.x();
				lidar_detects_object.kinematics.initial_pose_with_covariance.pose.orientation.y = qua.y();
				lidar_detects_object.kinematics.initial_pose_with_covariance.pose.orientation.z = qua.z();
				lidar_detects_object.kinematics.initial_pose_with_covariance.pose.orientation.w = qua.w();

				lidar_detects_object.kinematics.initial_twist_with_covariance.twist.linear.x = object.mobileye_obstacle.obstacle_relative_velocity_x;

				lidar_detects_object.shape.type = 0;
				lidar_detects_object.shape.dimensions.x = object.mobileye_obstacle.obstacle_length;
				lidar_detects_object.shape.dimensions.y = object.mobileye_obstacle.obstacle_width;

				detects.objects.push_back(lidar_detects_object);

				pub_mobileye_marker_->publish(*get_mobileye_marker_ptr(object));
			}*/
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

		pub_fusion_detect_ = create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>("out_fusion_detect", rclcpp::QoS(1));
		pub_debug_mobileye_array_ = create_publisher<detect_fusion_msgs::msg::MobileyeObstacleStampArray>("debug/mobileye_array", rclcpp::QoS(1));
		pub_mobileye_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>("mobileye_obstacle_marker", rclcpp::QoS(1));
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