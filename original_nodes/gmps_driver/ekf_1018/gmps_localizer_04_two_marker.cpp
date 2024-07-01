/*
横変位からx,y,yawへの変換
*/



#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "gmps_msgs/msg/gmps_detect.hpp" //横変位

#include <vector>

#define COND_STOP_SEARCH 0.04 //マーカーリストの全点探索を終了するかどうかの閾値　20cm以内のマーカがあれば終了する

struct marker_point
{
	int32_t mm_id; //磁気マーカの通し番号
	int32_t tag_id; //RFIDタグの番号 //まだ使わない
	int32_t mm_kind; //マーカ種別
	int32_t pole; //磁極 N=1 S=2
	double x; //埋設位置のX座標
	double y; //埋設位置のY座標
};

struct marker_info
{
	double x_marker; //磁気マーカ埋設座標X [m]
    double y_marker; //磁気マーカ埋設座標Y [m]
    int32_t mm_id; //磁気マーカ番号 []
    int32_t tag_id_table; //この磁気マーカと同時に検知されるはずのRFID番号 []
	int32_t tag_id_detected; //実際に検知したRFID番号
    int32_t mm_kind; //磁気マーカ種別 []
	int32_t pole; //磁極 N=1 S=2
    double side_diff; //磁気センサ横偏差 [m]
    double delay_dist; //磁気センサ検出遅延距離 [m]
	double x_prev_detected; //距離判定のためXYを保存 //debug 本当は速度で積分するだけでよい
	double y_prev_detected;
    double yaw_prev_detected; //直進判定のためyawを保存 //debug 本当は操舵角を見ればよい
}


class GMPSLocalizer : public rclcpp::Node
{
private:
    /* Publisher */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gmps_pose_;
    
    /* Subscriber */
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_prev_pose_;
    rclcpp::Subscription<gmps_msgs::msg::GmpsDetect>::SharedPtr sub_gmps_detect_;

	/* tf */
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

    /* parameters */
    double offset_gmps_; //座標基準点から磁気センサ中央までのオフセット[m]。プラスなら座標基準点より磁気センサ中央が前に位置する。磁気センサから見たときは座標基準点が後ろにある
    double expect_gmps_delay_dist_; //パッケージタイプは検知が10cmほど遅れる
    //double offset_rfid_; //座標基準点からRFIDリーダまでのオフセット[m]。プラスなら座標基準点よりRFIDリーダが前に位置する。RFIDリーダはGMPSセンサより前に位置する。
	double th_dist_double_marker_; //この距離以内なら二連マーカと判定
	double th_yaw_diff_double_marker_; //このyaw変化以内なら二連マーカと判定
    double th_association_dist_square_; //マーカーは1m離すので、0.5m以内のマーカーがあれば、それより近いマーカーは存在しない
    std::string marker_table_csv_name_; //マーカーテーブルのfilename
	double sigma_x_gmps_; //X座標の標準偏差
	double sigma_y_gmps_; //Y座標の標準偏差
	double sigma_theta_gmps_; //yawの標準偏差 //yawは観測しない。

	geometry_msgs::msg::PoseStamped prev_pose_; //自己位置の前回値
	double prev_yaw_; //previous yaw from prev pose
//	geometry_msgs::msg::Point predict_marker_pose_; //自己位置の前回値から予想するマーカー位置
//	geometry_msgs::msg::Point associated_marker_pose_; //対応付けで選んだマーカー位置
	geometry_msgs::msg::Pose measurement_pose_; //観測座標
    geometry_msgs::msg::PoseWithCovarianceStamped gmps_pose_; //output measurement pose for publish

	std::vector<marker_point> marker_table_; //磁気マーカ座標のテーブル
	marker_info marker_queue_[2]; //検知したマーカー情報を保存 queueを使うほどでもないので長さ２の配列
	bool f_double_marker_; //二連マーカとして処理するかどうかのフラグ

//	double side_diff_; //横変位


    enum MM_KIND{
        NONE = 0,
        SINGLE = 1,
        DOUBLE_START = 3,
        DOUBLE_END = 4,
    };


	// 外から受け取ったmsgを内部変数に保存するだけ
	void callbackPrevPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
        prev_pose_.pose.position.x = msg->pose.pose.position.x;
		prev_pose_.pose.position.y = msg->pose.pose.position.y;
		prev_pose_.pose.position.z = msg->pose.pose.position.z;
		prev_pose_.pose.orientation.x = msg->pose.pose.orientation.x;
		prev_pose_.pose.orientation.y = msg->pose.pose.orientation.y;
		prev_pose_.pose.orientation.z = msg->pose.pose.orientation.z;
		prev_pose_.pose.orientation.w = msg->pose.pose.orientation.w;

		//humbleだとtf2::getYaw関数でリンクエラーが発生するので、yawの取得方法を変更
		tf2::Quaternion qua(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		double roll, pitch, yaw;
		tf2::Matrix3x3(qua).getRPY(roll, pitch, yaw);
		prev_yaw_ = yaw;//tf2::getYaw(msg->pose.pose.orientation);
		
	}

	void callbackDetect(const gmps_msgs::msg::GmpsDetect::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "callback detet");

		//push queue
		marker_queue_[1] = marker_queue_[0];
		marker_queue_[0].side_diff = msg->side_diff;
		marker_queue_[0].pole = msg->pole;
		marker_queue_[0].mm_kind = msg->mm_kind;
		marker_queue_[0].delay_dist = msg->delay_dist;
		marker_queue_[0].x_prev_detected = prev_pose_.pose.pose.x; //二連マーカの判定のため
		marker_queue_[0].y_prev_detected = prev_pose_.pose.pose.y; //二連マーカの判定のため
		marker_queue_[0].yaw_prev_detected = prev_yaw_; //二連マーカの判定のため

		//prev poseによる磁気マーカの対応付け。磁気マーカの埋設位置が特定される。
		marker_association_by_prev_pose();

		//check dist and yaw between queue0 and queue1
		f_double_marker_ = false;
		if ((marker_queue_[0].mm_kind != 0) && (marker_queue_[1].mm_kind != 0)) 
		{ //queueが正常に2個あるときのみ二連マーカの判定をする
			f_double_marker_ = check_double_marker();
		}

		//二連マーカはX,Y,yaw
		if(f_double_marker_ == true)
		{
			measurement_double_marker();
		}
		else //単体マーカはXYのみ
		{
			measurement_single_marker();
		}


		//debug publishはmeasurementの中に入れてしまった方がいいかも sigmaを変える場合
		//poseのpublish
		gmps_pose_.header.frame_id = "map"; //frameIDはmap
		gmps_pose_.header.stamp = msg->header.stamp; //time stamp は磁気マーカを検知したタイミングなので、detect msgから継承
		gmps_pose_.pose.pose.position.x = measurement_pose_.position.x; //X, Yはcallbackで計算
		gmps_pose_.pose.pose.position.y = measurement_pose_.position.y;
		gmps_pose_.pose.pose.position.z = prev_pose_.pose.position.z; //Z, qはprev poseから継承
		gmps_pose_.pose.pose.orientation = measurement_pose_.orientation; //debug prev poseとdetectのstampが著しく乖離した場合は？
		gmps_pose_.pose.covariance[6 * 0 + 0] = sigma_x_gmps_*sigma_x_gmps_; //x*x
		gmps_pose_.pose.covariance[6 * 1 + 1] = sigma_y_gmps_*sigma_y_gmps_; //y*y
		gmps_pose_.pose.covariance[6 * 5 + 5] = sigma_theta_gmps_*sigma_theta_gmps_; //yaw*yaw
		
		pub_gmps_pose_->publish(gmps_pose_);

		geometry_msgs::msg::TransformStamped transformStamped;
		transformStamped.header.stamp = this->now();
		transformStamped.header.frame_id = "map";
		transformStamped.child_frame_id = "gmps_link";
		transformStamped.transform.translation.x = gmps_pose_.pose.pose.position.x;
		transformStamped.transform.translation.y = gmps_pose_.pose.pose.position.y;
		transformStamped.transform.translation.z = gmps_pose_.pose.pose.position.z;

		transformStamped.transform.rotation.x = gmps_pose_.pose.pose.orientation.x;
		transformStamped.transform.rotation.y = gmps_pose_.pose.pose.orientation.y;
		transformStamped.transform.rotation.z = gmps_pose_.pose.pose.orientation.z;
		transformStamped.transform.rotation.w = gmps_pose_.pose.pose.orientation.w;

		tf_br_->sendTransform(transformStamped);
	}

	void marker_association_by_prev_pose()
	{
        //baselinkからsensor, markerまでの座標変換
			//tfを使うのが望ましいかもだが、一旦よい
		geometry_msgs::msg::Point predict_marker_pose; //自己位置の前回値から予想するマーカー位置
		predict_marker_pose.x = prev_pose_.pose.position.x
		 + (offset_gmps_ - marker_queue_[0].delay_dist) * cos(prev_yaw_) + marker_queue_[0].side_diff * sin(prev_yaw_);
		predict_marker_pose.y = prev_pose_.pose.position.y
		 + (offset_gmps_ - marker_queue_[0].delay_dist) * sin(prev_yaw_) - marker_queue_[0].side_diff * cos(prev_yaw_);
		
		//association
		double min_dist_square = 100000;
		int min_i = 100000;
		double dist_square;
		for (int i=0; i < marker_table_.size(); i++)
		{
			//極性による探索を先に行う　多少の高速化
			if (marker_queue_[0].pole != marker_table_[i].pole) //極性が一致しないときはスキップ
			{
				continue;
			}

			dist_square = (marker_table_[i].x - predict_marker_pose.x)*(marker_table_[i].x - predict_marker_pose.x)
			 + (marker_table_[i].y - predict_marker_pose.y)*(marker_table_[i].y - predict_marker_pose.y);

			// マーカーリストの全点探索
			if (dist_square < min_dist_square)
			{
				min_dist_square = dist_square;
				min_i = i;

				// かなり近いマーカがあればそこで終了
				if(min_dist_square < COND_STOP_SEARCH)
				{
					RCLCPP_INFO(this->get_logger(), "terminate list search by %f",COND_STOP_SEARCH);
					break;
				}
			}
		}

		//　エラー判定　閾値以内のマーカが見つかっていないなら以降の処理は行わない
		if (min_dist_square > th_association_dist_square_)
		{
			RCLCPP_ERROR(this->get_logger(),
			 "Ignore detect. there are no markers within %.2f neighborhood of prev_x=%.2f, prev_y=%.2f",
			  th_association_dist_square_, predict_marker_pose.x, predict_marker_pose.y);
			return;
		}

		// return
		marker_queue_[0].x_marker = marker_table_[min_i].x;
		marker_queue_[0].y_marker = marker_table_[min_i].y;
		marker_queue_[0].mm_id = marker_table_[min_i].mm_id;
		marker_queue_[0].tag_id_expected = marker_table_[min_i].tag_id;
		RCLCPP_INFO(this->get_logger(),
		 "push marker queue with mm_id=%d, tag_id=%d, X=%lf, Y=%lf",
		   marker_queue_[0].mm_id, marker_queue_[0].tag_id_expected,
		    marker_queue_[0].x_marker, marker_queue_[0].y_marker);

	}

	// 二連マーカとして処理するかどうかの判定
	bool check_double_marker()
	{
		//一個目のマーカから二個目のマーカを検知するまでの走行距離が閾値以内であること
		//debug 速度の積分によって判定したいが、prev poseしかないため
		//debug prev pose同士の距離によって判定する
		const double dist_between_markers = sqrt(
			(marker_queue_[0].x_prev_detected - marker_queue_[1].x_prev_detected)^2 + 
			(marker_queue_[0].y_prev_detected - marker_queue_[1].y_prev_detected)^2
		);
		//一個目のマーカと二個目のマーカでyawが大きく変化していないこと
		const double yaw_diff_between_markers = 
		 marker_queue_[0].yaw_prev_detected - marker_queue_[1].yaw_prev_detected;

		return ((dist_between_markers < th_dist_double_marker_) && 
				(abs(yaw_diff_between_markers) < th_yaw_diff_double_marker_));
	}

	void measurement_single_marker()
	{
		//markerからsensor, baselinkまでの変換
		measurement_pose_.position.x = marker_queue_[0].x_marker 
		 - (offset_gmps_ - marker_queue_[0].delay_dist) * cos(prev_yaw_) - marker_queue_[0].side_diff * sin(prev_yaw_);
		measurement_pose_.position.y = marker_queue_[0].y_marker 
		 - (offset_gmps_ - marker_queue_[0].delay_dist) * sin(prev_yaw_) + marker_queue_[0].side_diff * cos(prev_yaw_);
		//yawはprev_poseから継承
		measurement_pose_.orientation = prev_pose.pose.orientation;
	}

	void measurement_double_marker()
	{
		//磁気マーカ間のyawを計算 1個目[1]を基準とした2個目[0]へのyaw
		const double dx = marker_queue_[0].x_marker - marker_queue_[1].x_marker;
		const double dy = marker_queue_[0].y_marker - marker_queue_[1].y_marker;
		const double yaw_marker = atan2(dy, dx);
		//二つの横変位からdthetaを計算
		//マーカ間距離には測量による真値を使用する。prev_poseの比較よりも正しいはず
		const double dist_marker = sqrt(dx^2 + dy^2);
		const double dtheta = asin((marker_queue_[0].side_diff - marker_queue_[1].side_diff) / dist_marker );
		//その合計が現在のyaw
		const double yaw_gmps = yaw_marker + dtheta;
		//markerからsensor, baselinkまでの変換
		measurement_pose_.position.x = marker_queue_[0].x_marker 
		 - (offset_gmps_ - marker_queue_[0].delay_dist) * cos(yaw_gmps) - marker_queue_[0].side_diff * sin(yaw_gmps);
		measurement_pose_.position.y = marker_queue_[0].y_marker 
		 - (offset_gmps_ - marker_queue_[0].delay_dist) * sin(yaw_gmps) + marker_queue_[0].side_diff * cos(yaw_gmps);
		//quaternion
		tf2::Quaternion qua;
		qua.setRPY(0,0,yaw_gmps);
		measurement_pose_.orientation = qua;

		//debug covarianceも1と2で変えたい
	}

	//debug loadのところはがっつり手直しだろうけど、一旦コピペで動けばいいよ
	void load_marker_csv(const char *csvname)
	{
		//fp
		if (NULL == csvname)
		{
			RCLCPP_ERROR(this->get_logger(),"csvname is empty");
			exit(1);
		}
		char buf[256];
		
		FILE *fp = fopen(csvname, "r");
		if (fp == NULL)
		{
			RCLCPP_ERROR(this->get_logger(),"could not open marker_table file %s", csvname);
			exit(1);
		}

		fscanf(fp, "%s\n", buf); // headerは捨てる

		int32_t ret = 0;
		int32_t num[4];
		double data[2];
		marker_point record; //vector代入用の一時変数
		while((ret = fscanf(fp, "%d, %d, %d, %d, %lf, %lf\n", 
		&num[0], &num[1], &num[2], &num[3], &data[0], &data[1] )) != EOF)
			{
				record.mm_id = num[0]; 
				record.tag_id = num[1];
				record.mm_kind = num[2];
				record.pole = num[3];
				record.x = data[0];
				record.y = data[1];
				marker_table_.push_back(record);
			}
		fclose(fp);
	}


public:
	GMPSLocalizer(const rclcpp::NodeOptions &node_options)
    	: rclcpp::Node("gmps_localizer", node_options)
	{
        /* parameters */
        this->declare_parameter("offset_gmps", 1.5);
        offset_gmps_ = this->get_parameter("offset_gmps").as_double();
        this->declare_parameter("expect_gmps_delay_dist", 0.2);
        expect_gmps_delay_dist_ = this->get_parameter("expect_gmps_delay_dist").as_double();
        this->declare_parameter("th_association_dist_square", 0.25);
        th_association_dist_square_ = this->get_parameter("th_association_dist_square").as_double();
		this->declare_parameter("th_dist_double_marker", 2.5);
        th_dist_double_marker_ = this->get_parameter("th_dist_double_marker").as_double();
		this->declare_parameter("th_yaw_diff_double_marker", 0.05);
        th_yaw_diff_double_marker_ = this->get_parameter("th_yaw_diff_double_marker").as_double();
        this->declare_parameter("marker_table_csv_name", "gmps_driver/DATA/marker.csv");
        marker_table_csv_name_ = this->get_parameter("marker_table_csv_name").as_string();
		this->declare_parameter("sigma_x_gmps", 0.07);
		sigma_x_gmps_ = this->get_parameter("sigma_x_gmps").as_double();
		this->declare_parameter("sigma_y_gmps", 0.07);
		sigma_y_gmps_ = this->get_parameter("sigma_y_gmps").as_double();
		this->declare_parameter("sigma_theta_gmps", 0.1);
		sigma_theta_gmps_ = this->get_parameter("sigma_theta_gmps").as_double();
		


        /* Publisher */
        pub_gmps_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gmps_pose", rclcpp::QoS(1));

        /* Subscriber */
        sub_prev_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "prev_pose", rclcpp::QoS(1), std::bind(&GMPSLocalizer::callbackPrevPose, this, std::placeholders::_1));
        sub_gmps_detect_ = this->create_subscription<gmps_msgs::msg::GmpsDetect>(
            "gmps_detect", rclcpp::SensorDataQoS(), std::bind(&GMPSLocalizer::callbackDetect, this, std::placeholders::_1));

		tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    		std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

		//marker.csvのロード
		load_marker_csv( (char*)marker_table_csv_name_.c_str() );

		RCLCPP_INFO(this->get_logger(), "constructor end");
	}
};








int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<GMPSLocalizer>(node_options);

  rclcpp::spin(node);

  return 0;
}