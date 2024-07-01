#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

const size_t predicted_path_count = 100UL;
using sequence_predected_path = rosidl_runtime_cpp::BoundedVector<autoware_auto_perception_msgs::msg::PredictedPath, predicted_path_count, std::allocator<autoware_auto_perception_msgs::msg::PredictedPath>>;
const size_t path_count = 100UL;
using sequence_path = rosidl_runtime_cpp::BoundedVector<geometry_msgs::msg::Pose, path_count, std::allocator<geometry_msgs::msg::Pose>>;

//bufにdataをbyte単位でpushする。bufの最初はデータ全長なので、bufの最初にpushしたバイト数を足す
template<class T>
void dataPush(std::vector<uint8_t> &buf, const T data)
{
	const uint8_t* dataptr = (uint8_t*)&data;
	for(size_t i=0; i<sizeof(T); i++) buf.push_back(dataptr[i]);
	size_t* count = reinterpret_cast<size_t*>(buf.data());
	*count += sizeof(T);
}

class Ros2DetectMqttPub : public rclcpp::Node
{
private:
	//-------------mqtt関連-------------
	bool mosq_success_flag_;//mosquitto正常起動フラグ
	struct mosquitto *mosq_;//mosquittoインスタンス

	//Brokerとの接続成功時に実行されるcallback関数
	static void mqttOnConnect(struct mosquitto *mosq, void *obj, int result)
	{
	}

	//Brokerとの接続を切断した時に実行されるcallback関数
	static void mqttOnDisconnect(struct mosquitto *mosq, void *obj, int rc)
	{
	}

	//BROKEにMESSAGE送信後に実行されるcallback関数
	static void mqttOnPublish(struct mosquitto *mosq, void *userdata, int mid)
	{

	}

	//-------------ros2関連-------------
	rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr sub_detect_object_;//center pointで検出されたオブジェクト情報

	//center pointオブジェクトをsubscribeするコールバック
	void callbackDetectObjects(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr)
	{
		std::vector<uint8_t> pubdata(sizeof(size_t));
		//mqttで送信するデータの最初は送るデータの全長バイト数にしたいので、全長バイト数自身の変数サイズを予め足しておく
		{
			size_t* count = reinterpret_cast<size_t*>(pubdata.data());
			*count += sizeof(size_t);
		}

		const std_msgs::msg::Header &header = objects_ptr->header;
		const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> &objects = objects_ptr->objects;

		//------------center pointのオブジェクト群を1列のbyteデータにする(開始)-----------
		dataPush<size_t>(pubdata, objects.size());
		for(const autoware_auto_perception_msgs::msg::PredictedObject obj : objects)
		{
			const unique_identifier_msgs::msg::UUID &obj_id = obj.object_id;
			for(size_t i=0; i<obj_id.uuid.size(); i++)
				dataPush<uint8_t>(pubdata, obj_id.uuid[i]);

			dataPush<float>(pubdata, obj.existence_probability);

			const std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> &classification = obj.classification;
			dataPush<size_t>(pubdata, classification.size());

			for(size_t ci=0; ci<classification.size(); ci++)
			{
				dataPush<uint8_t>(pubdata, classification[ci].label);
				dataPush<float>(pubdata, classification[ci].probability);
			}

			const autoware_auto_perception_msgs::msg::PredictedObjectKinematics &kinematics = obj.kinematics;
			{
				const size_t conv_size = 36;
				const geometry_msgs::msg::PoseWithCovariance &pose_with_covariance = kinematics.initial_pose_with_covariance;
				dataPush<double>(pubdata, pose_with_covariance.pose.position.x);
				dataPush<double>(pubdata, pose_with_covariance.pose.position.y);
				dataPush<double>(pubdata, pose_with_covariance.pose.position.z);
				dataPush<double>(pubdata, pose_with_covariance.pose.orientation.x);
				dataPush<double>(pubdata, pose_with_covariance.pose.orientation.y);
				dataPush<double>(pubdata, pose_with_covariance.pose.orientation.z);
				dataPush<double>(pubdata, pose_with_covariance.pose.orientation.w);
				const std::array<double, conv_size> &pose_cov = pose_with_covariance.covariance;
				for(size_t pi=0; pi<conv_size; pi++) dataPush<double>(pubdata, pose_cov[pi]);

				const geometry_msgs::msg::TwistWithCovariance &twist_with_covariance = kinematics.initial_twist_with_covariance;
				dataPush<double>(pubdata, twist_with_covariance.twist.linear.x);
				dataPush<double>(pubdata, twist_with_covariance.twist.linear.y);
				dataPush<double>(pubdata, twist_with_covariance.twist.linear.z);
				dataPush<double>(pubdata, twist_with_covariance.twist.angular.x);
				dataPush<double>(pubdata, twist_with_covariance.twist.angular.y);
				dataPush<double>(pubdata, twist_with_covariance.twist.angular.z);
				const std::array<double, conv_size> &twist_cov = twist_with_covariance.covariance;
				for(size_t pi=0; pi<conv_size; pi++) dataPush<double>(pubdata, twist_cov[pi]);

				const geometry_msgs::msg::AccelWithCovariance &acc_with_covariance = kinematics.initial_acceleration_with_covariance;
				dataPush<double>(pubdata, acc_with_covariance.accel.linear.x);
				dataPush<double>(pubdata, acc_with_covariance.accel.linear.y);
				dataPush<double>(pubdata, acc_with_covariance.accel.linear.z);
				dataPush<double>(pubdata, acc_with_covariance.accel.angular.x);
				dataPush<double>(pubdata, acc_with_covariance.accel.angular.y);
				dataPush<double>(pubdata, acc_with_covariance.accel.angular.z);
				const std::array<double, conv_size> &acc_cov = acc_with_covariance.covariance;
				for(size_t pi=0; pi<conv_size; pi++) dataPush<double>(pubdata, acc_cov[pi]);
			
				const sequence_predected_path &predicted_path = kinematics.predicted_paths;
				dataPush<size_t>(pubdata, predicted_path.size());
				for(size_t pi=0; pi<predicted_path.size(); pi++)
				{
					const autoware_auto_perception_msgs::msg::PredictedPath &pred_path = predicted_path[pi];
					const sequence_path &path = pred_path.path;
					dataPush<size_t>(pubdata, path.size());
					//RCLCPP_INFO(this->get_logger(), "%ld", path.size());
					for(size_t i=0; i<path.size(); i++)
					{
						dataPush<double>(pubdata, path[i].position.x);
						dataPush<double>(pubdata, path[i].position.y);
						dataPush<double>(pubdata, path[i].position.z);
						dataPush<double>(pubdata, path[i].orientation.x);
						dataPush<double>(pubdata, path[i].orientation.y);
						dataPush<double>(pubdata, path[i].orientation.z);
						dataPush<double>(pubdata, path[i].orientation.w);
					}

					dataPush<int32_t>(pubdata, pred_path.time_step.sec);
					dataPush<uint32_t>(pubdata, pred_path.time_step.nanosec);

					dataPush<float>(pubdata, pred_path.confidence);
				}

				/*for(size_t pi=0; pi<kinematics.predicted_paths.size(); pi++)
				{
					for(size_t i=0; i<kinematics.predicted_paths[pi].path.size(); i++)
					{
						dataPush<double>(pubdata, kinematics.predicted_paths[pi].path[i].position.x);
						dataPush<double>(pubdata, kinematics.predicted_paths[pi].path[i].position.y);
						dataPush<double>(pubdata, kinematics.predicted_paths[pi].path[i].position.z);
						dataPush<double>(pubdata, kinematics.predicted_paths[pi].path[i].orientation.x);
						dataPush<double>(pubdata, kinematics.predicted_paths[pi].path[i].orientation.y);
						dataPush<double>(pubdata, kinematics.predicted_paths[pi].path[i].orientation.z);
						dataPush<double>(pubdata, kinematics.predicted_paths[pi].path[i].orientation.w);
					}

					dataPush<int32_t>(pubdata, kinematics.predicted_paths[pi].time_step.sec);
					dataPush<uint32_t>(pubdata, kinematics.predicted_paths[pi].time_step.nanosec);

					dataPush<float>(pubdata, kinematics.predicted_paths[pi].confidence);
				}*/
			}

			const autoware_auto_perception_msgs::msg::Shape sharp = obj.shape;
			{
				dataPush<uint8_t>(pubdata, sharp.type);
				const geometry_msgs::msg::Polygon &footprint = sharp.footprint;
				dataPush<size_t>(pubdata, footprint.points.size());
				for(int pi=0; pi<footprint.points.size(); pi++)
				{
					dataPush<float>(pubdata, footprint.points[pi].x);
					dataPush<float>(pubdata, footprint.points[pi].y);
					dataPush<float>(pubdata, footprint.points[pi].z);
				}
				dataPush<double>(pubdata, sharp.dimensions.x);
				dataPush<double>(pubdata, sharp.dimensions.y);
				dataPush<double>(pubdata, sharp.dimensions.z);
			}
		}
		//------------center pointのオブジェクト群を1列のbyteデータにする(終了)-----------

		std::cout << "pubsize," << pubdata.size() << std::endl;
		int mos_ret = mosquitto_publish(this->mosq_, nullptr, "/ros2_detect", pubdata.size(), pubdata.data(), 0, false);
		if(mos_ret == MOSQ_ERR_SUCCESS) RCLCPP_INFO(this->get_logger(), "mosquitto pub success");
		else RCLCPP_INFO(this->get_logger(), "mosquitto pub failure");
	}
public:
	Ros2DetectMqttPub()
		: rclcpp::Node("ros2_detect_mqtt_pub")
		, mosq_(nullptr)
		, mosq_success_flag_(false)
	{
		std::string ip = this->declare_parameter<std::string>("ip", "192.168.12.11");

		sub_detect_object_ = create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
			"/perception/object_recognition/objects", rclcpp::QoS{1}, std::bind(&Ros2DetectMqttPub::callbackDetectObjects, this, std::placeholders::_1));

		//mosquit初期化
		int mos_ret = mosquitto_lib_init();
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			RCLCPP_INFO(this->get_logger(), "error : init");
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoインスタンスの作成
		std::string mqtt_node_name = "center_point_mqtt_pub";
		bool clean_session = true;
		this->mosq_ = mosquitto_new(mqtt_node_name.c_str(), clean_session, NULL);
		if(this->mosq_ == nullptr)
		{
			RCLCPP_INFO(this->get_logger(), "error : mosquitto_new");
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoコールバックのセット
		mosquitto_connect_callback_set(this->mosq_, Ros2DetectMqttPub::mqttOnConnect);
		mosquitto_disconnect_callback_set(this->mosq_, Ros2DetectMqttPub::mqttOnDisconnect);
		mosquitto_publish_callback_set(this->mosq_, Ros2DetectMqttPub::mqttOnPublish);

		//mqttサーバーにコネクト
		mos_ret = mosquitto_connect(this->mosq_, ip.c_str(), 1883, 60);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			RCLCPP_INFO(this->get_logger(), "error : mosquitto_connect");
			mosquitto_lib_cleanup();
			return;
		}

		mosq_success_flag_ = true;
	}

	bool mosqSuccess() {return mosq_success_flag_;}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<Ros2DetectMqttPub> detect = std::make_shared<Ros2DetectMqttPub>();
	if(detect->mosqSuccess() == true) rclcpp::spin(detect);
	else RCLCPP_INFO(detect->get_logger(), "mosquitto failure. node close");
	rclcpp::shutdown();
	return 0;
}