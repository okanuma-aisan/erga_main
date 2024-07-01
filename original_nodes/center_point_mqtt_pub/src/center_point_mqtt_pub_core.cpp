#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

//bufにdataをbyte単位でpushする。bufの最初はデータ全長なので、bufの最初にpushしたバイト数を足す
template<class T>
void dataPush(std::vector<uint8_t> &buf, const T data)
{
	const uint8_t* dataptr = (uint8_t*)&data;
	for(size_t i=0; i<sizeof(T); i++) buf.push_back(dataptr[i]);
	size_t* count = reinterpret_cast<size_t*>(buf.data());
	*count += sizeof(T);
}

class CenterPointMqttPub : public rclcpp::Node
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
	rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr sub_center_point_object_;//center pointで検出されたオブジェクト情報

	//center pointオブジェクトをsubscribeするコールバック
	void callbackCenterPointObjects(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr objects_ptr)
	{
		std::vector<uint8_t> pubdata(sizeof(size_t));
		//mqttで送信するデータの最初は送るデータの全長バイト数にしたいので、全長バイト数自身の変数サイズを予め足しておく
		{
			size_t* count = reinterpret_cast<size_t*>(pubdata.data());
			*count += sizeof(size_t);
		}

		const std_msgs::msg::Header &header = objects_ptr->header;
		const std::vector<autoware_auto_perception_msgs::msg::DetectedObject> &objects = objects_ptr->objects;


		//------------center pointのオブジェクト群を1列のbyteデータにする(開始)-----------
		dataPush<size_t>(pubdata, objects.size());
		for(const autoware_auto_perception_msgs::msg::DetectedObject obj : objects)
		{
			dataPush<float>(pubdata, obj.existence_probability);
			const std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> &classification = obj.classification;
			dataPush<size_t>(pubdata, classification.size());
			for(size_t ci=0; ci<classification.size(); ci++)
			{
				dataPush<uint8_t>(pubdata, classification[ci].label);
				dataPush<float>(pubdata, classification[ci].probability);
			}

			const autoware_auto_perception_msgs::msg::DetectedObjectKinematics &kinematics = obj.kinematics;
			{
				const size_t conv_size = 36;
				const geometry_msgs::msg::PoseWithCovariance &pose_with_covariance = kinematics.pose_with_covariance;
				dataPush<double>(pubdata, pose_with_covariance.pose.position.x);
				dataPush<double>(pubdata, pose_with_covariance.pose.position.y);
				dataPush<double>(pubdata, pose_with_covariance.pose.position.z);
				dataPush<double>(pubdata, pose_with_covariance.pose.orientation.x);
				dataPush<double>(pubdata, pose_with_covariance.pose.orientation.y);
				dataPush<double>(pubdata, pose_with_covariance.pose.orientation.z);
				dataPush<double>(pubdata, pose_with_covariance.pose.orientation.w);
				const std::array<double, conv_size> &pose_cov = pose_with_covariance.covariance;
				for(size_t pi=0; pi<conv_size; pi++) dataPush<double>(pubdata, pose_cov[pi]);

				dataPush<bool>(pubdata, kinematics.has_position_covariance);
				dataPush<uint8_t>(pubdata, kinematics.orientation_availability);

				const geometry_msgs::msg::TwistWithCovariance &twist_with_covariance = kinematics.twist_with_covariance;
				dataPush<double>(pubdata, twist_with_covariance.twist.linear.x);
				dataPush<double>(pubdata, twist_with_covariance.twist.linear.y);
				dataPush<double>(pubdata, twist_with_covariance.twist.linear.z);
				dataPush<double>(pubdata, twist_with_covariance.twist.angular.x);
				dataPush<double>(pubdata, twist_with_covariance.twist.angular.y);
				dataPush<double>(pubdata, twist_with_covariance.twist.angular.z);
				const std::array<double, conv_size> &twist_cov = twist_with_covariance.covariance;
				for(size_t pi=0; pi<conv_size; pi++) dataPush<double>(pubdata, twist_cov[pi]);

				dataPush<bool>(pubdata, kinematics.has_twist);
				dataPush<bool>(pubdata, kinematics.has_twist_covariance);
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
		int mos_ret = mosquitto_publish(this->mosq_, nullptr, "/centerpoint_data", pubdata.size(), pubdata.data(), 0, false);
		if(mos_ret == MOSQ_ERR_SUCCESS) std::cout << "mosquitto pub success" << std::endl;
		else std::cout << "mosquitto pub failure" << std::endl;
	}
public:
	CenterPointMqttPub()
		: rclcpp::Node("center_point_mqtt_pub")
		, mosq_(nullptr)
		, mosq_success_flag_(false)
	{
		sub_center_point_object_ = create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
			"/centerpoint/objects", rclcpp::QoS{1}, std::bind(&CenterPointMqttPub::callbackCenterPointObjects, this, std::placeholders::_1));

		//mosquit初期化
		int mos_ret = mosquitto_lib_init();
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : init" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoインスタンスの作成
		std::string mqtt_node_name = "center_point_mqtt_pub";
		bool clean_session = true;
		this->mosq_ = mosquitto_new(mqtt_node_name.c_str(), clean_session, NULL);
		if(this->mosq_ == nullptr)
		{
			std::cout << "error : mosquitto_new" << std::endl;
			mosquitto_lib_cleanup();
			return;
		}

		//mosquittoコールバックのセット
		mosquitto_connect_callback_set(this->mosq_, CenterPointMqttPub::mqttOnConnect);
		mosquitto_disconnect_callback_set(this->mosq_, CenterPointMqttPub::mqttOnDisconnect);
		mosquitto_publish_callback_set(this->mosq_, CenterPointMqttPub::mqttOnPublish);

		//mqttサーバーにコネクト
		mos_ret = mosquitto_connect(this->mosq_, "192.168.12.5", 1883, 60);
		if(mos_ret != MOSQ_ERR_SUCCESS)
		{
			std::cout << "error : mosquitto_connect" << std::endl;
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
	std::shared_ptr<CenterPointMqttPub> cpmp = std::make_shared<CenterPointMqttPub>();
	if(cpmp->mosqSuccess() == true) rclcpp::spin(cpmp);
	else std::cout << "mosquitto failure. node close" << std::endl;
	return 0;
}