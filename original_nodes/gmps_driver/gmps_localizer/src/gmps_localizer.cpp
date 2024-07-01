/*
横変位からx,y,yawへの変換
*/

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

//#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> //マーカを探索するための位置
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp> //走行距離計算のため追加 v2.2.0
#include "gmps_msgs/msg/gmps_detect.hpp" //GMPS横変位
#include "gmps_msgs/msg/gmps_info.hpp" //解析用のデータ
#include "gmps_msgs/msg/rfid.hpp" //RFIDのデータ


#define BACK_MARGIN 0 //探索範囲を後ろ側にどれだけ下げるか
#define RFID_QUEUE_SIZE 10 //RFIDの検知情報を貯めこむqueueのサイズ //note10を超えることはまずない。余裕を見ても5ぐらいでよいのでは

#define SHOW_DEBUG_INFO 1 //DEBUG_INFOを有効
#define DEBUG_INFO(...) {if (SHOW_DEBUG_INFO) {RCLCPP_INFO(__VA_ARGS__);}}

enum QUEUE_STAMP
{ //可読性のためenumを定義
	NOW = 0, //直近の検知が0
	PREV = 1 //前回の検知が1
};
struct marker_point
{ //メモリ節約のためマーカーテーブルのデータ型を縮小　整数6桁必要なのでuint16では足りない。
	uint32_t mm_id; //磁気マーカの通し番号 //debug mm_idは不要。tag_idのみでよい
	uint32_t tag_id; //RFIDタグの番号 //まだ使わない
	uint8_t mm_kind; //マーカ種別
	uint8_t pole; //磁極 N=1 S=2
	float x; //埋設位置のX座標
	float y; //埋設位置のY座標
};

struct marker_info
{
	double x_marker; //磁気マーカ埋設座標X [m]
    double y_marker; //磁気マーカ埋設座標Y [m]
    int32_t mm_id; //磁気マーカ番号 []
    int32_t tag_id_table; //この磁気マーカに対応して検知されるはずのRFID番号 []
	int32_t tag_id_detected; //実際に検知したRFID番号
    int32_t mm_kind; //磁気マーカ種別 []
	int32_t pole; //磁極 N=1 S=2
    double side_diff; //磁気センサ横偏差 [m]
    double delay_dist; //磁気センサ検出遅延距離 [m]
    double yaw_prev_detected; //直進判定のためyawを保存 //debug 本当は操舵角を見ればよい
	double mileage; //走行距離判定のため追加 v2.2.0
	int32_t min_i; //次回探索の初期値にするための行番号
};

struct rfid_info
{
	int32_t tag_id_detected;
	double mileage; //走行距離判定のため追加 v2.2.0
};


class GMPSLocalizer : public rclcpp::Node
{
private:
    /* Publisher */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gmps_pose_;
    rclcpp::Publisher<gmps_msgs::msg::GmpsInfo>::SharedPtr pub_gmps_info_;    
    /* Subscriber */
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_prev_pose_;
    rclcpp::Subscription<gmps_msgs::msg::GmpsDetect>::SharedPtr sub_gmps_detect_;
	rclcpp::Subscription<gmps_msgs::msg::Rfid>::SharedPtr sub_rfid_;
	rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_velocity_; //走行距離の計算のため追加 v2.2.0
	/* Timer */
	rclcpp::TimerBase::SharedPtr timer_; //走行距離の計算のため追加 v2.2.0
	/* tf */
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;


    /* parameters */
	bool param_enable_pole_;//
	bool param_enable_rfid_;
    double param_tf_x_; //座標基準点から磁気センサ中央までのオフセット[m]。プラスなら座標基準点より磁気センサ中央が前に位置する。磁気センサから見たときは座標基準点が後ろにある
	double param_tf_y_; //+のとき車両中心に対して磁気センサ中心が右側にある debug tfの向きと逆？GMPSの向きとは一致する
	double param_tf_yaw_; //+のときCCW
    double param_tf_rfid_x_; //座標基準点からRFIDリーダまでのオフセット[m]。プラスなら座標基準点よりRFIDリーダが前に位置する。RFIDリーダはGMPSセンサより前に位置する。
	double param_th_rfid_detect_range_m_; //RFIDの紐づけ許容誤差[m]
    double param_th_association_error_dist_m_; //自己位置から予測したマーカ位置と実際のマーカ敷設位置との許容誤差[m]
    double param_th_association_break_dist_m_; //探索を打ち切る閾値[m]。マーカーの最小設置間隔は決まっているので、ある程度近いマーカが見つかったらそれ以上探索する必要はない //debug ここの閾値は一つに統一したい
	double param_th_dist_double_marker_m_; //この距離以内なら二連マーカと判定
	double param_th_yaw_diff_double_marker_rad_; //このyaw変化以内なら二連マーカと判定
    std::string marker_table_csv_name_; //マーカーテーブルのfilename
	double sigma_x_gmps_; //X座標の標準偏差
	double sigma_y_gmps_; //Y座標の標準偏差
	double sigma_theta_gmps_; //yawの標準偏差 //yawは観測しない。

	geometry_msgs::msg::PoseWithCovarianceStamped prev_pose_; //自己位置の前回値 //debubg stampを使っている形跡がない
	double prev_yaw_; //previous yaw from prev pose
	geometry_msgs::msg::PoseWithCovarianceStamped measurement_pose_; //観測座標
	rclcpp::Time gmps_stamp_; //GMPS検知のタイムスタンプ
	double vx_mps_=0.0; //velocity [m/s]
	double mileage_; //走行距離[m] バックのときは走行距離が減るのでエラー
	rclcpp::Time prev_time_;
	double dt_;


	std::vector<marker_point> marker_table_; //磁気マーカ座標のテーブル
	marker_info marker_queue_[2]={}; //検知したマーカー情報を保存 queueを使うほどでもないので長さ２の配列
	std::queue<rfid_info> rfid_queue_; //RFIDタグの検知情報のqueue
	bool f_double_marker_; //二連マーカとして処理するかどうかのフラグ
	bool f_marker_association_success_;//RFID or poseによる対応付けに成功したフラグ
	bool f_pose_covariance_high_; //DRが続いてposeの信頼度が低い場合
	bool f_valid_rfid_detected_; //RFIDキューから対応するものを見つけたフラグ
	bool f_prev_pose_once_received_=false; //prev_poseを1回以上受信したフラグ 上位のEKFが初期位置を与えるまで何もpubしない場合のため
	bool f_ignore_pose_; //poseに判定をskipするためのフラグ
	double min_rfid_assoc_; //RFIDを検知してから磁気マーカ検知するまでの距離　許容最小
	double max_rfid_assoc_; //RFIDを検知してから磁気マーカ検知するまでの距離　許容最大

#if 0 //debug mm_kindを使うあてがない
    enum MM_KIND{
        NONE = 0,
        SINGLE = 1,
        DOUBLE_START = 3,
        DOUBLE_END = 4,
    };
#endif

	//バックのとき走行距離が減る
	void callbackVelocity(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
	{
		vx_mps_ = msg->twist.twist.linear.x;
	}

	//走行距離を計算するだけのタイマー　粗めの周期で更新しておき、RFIDかGMPSを検知したタイミングで都度更新する。
	void callbackTimer()
	{
		update_mileage();
	}

	// 外から受け取ったmsgを内部変数に保存するだけ
	void callbackPrevPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
        prev_pose_.pose.pose.position.x = msg->pose.pose.position.x;
		prev_pose_.pose.pose.position.y = msg->pose.pose.position.y;
		prev_pose_.pose.pose.position.z = msg->pose.pose.position.z;
		prev_pose_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
		prev_pose_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
		prev_pose_.pose.pose.orientation.z = msg->pose.pose.orientation.z;
		prev_pose_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
		prev_pose_.pose.covariance[6 * 0 + 0] = msg->pose.covariance[6 * 0 + 0]; //初期位置推定かどうかを判別するためにcovarianceを取得する

		//humbleだとtf2::getYaw関数でリンクエラーが発生するので、yawの取得方法を変更
		tf2::Quaternion qua(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		double roll, pitch, yaw;
		tf2::Matrix3x3(qua).getRPY(roll, pitch, yaw);
		prev_yaw_ = yaw;
		//prev_yaw_ = tf2::getYaw(msg->pose.pose.orientation);

		f_prev_pose_once_received_ = true; //一度もprev_poseを受信しなければfalseのまま
	}

	void callbackRfid(const gmps_msgs::msg::Rfid::SharedPtr msg)
	{
		DEBUG_INFO(this->get_logger(), "====callback rfid====");
		//check empty and duplicate
		if (rfid_queue_.empty() == false)
		{
			if (rfid_queue_.back().tag_id_detected == msg->tag_no)
			{
				DEBUG_INFO(this->get_logger(), "ignore same rfid tag");
				//同じ番号であれば無視
				return;
			}
			else if (vx_mps_ < 0.0) //バック中の検知は無視する。停止中はOK
			{
				DEBUG_INFO(this->get_logger(), "ignore this rfid tag. detect RFID tag while reverse driving");
				return;
			}
		}

		//push queue
		update_mileage(); //mileageの更新
		rfid_info new_info;
		new_info.tag_id_detected = msg->tag_no;
		new_info.mileage = mileage_;
		rfid_queue_.push(new_info);
		DEBUG_INFO(this->get_logger(), "push rfid queue with tag_id=%d at mileage=%.2f", new_info.tag_id_detected, new_info.mileage);


		//check overflow
		if (rfid_queue_.size() > RFID_QUEUE_SIZE)
		{
			//queueが大きくなりすぎていたら古いやつを消す
			DEBUG_INFO(this->get_logger(), "queue size over. erase old rfid queue");
			rfid_queue_.pop();
		}
	}

	/* 磁気マーカ観測座標のpublish */
	void callbackDetect(const gmps_msgs::msg::GmpsDetect::SharedPtr msg)
	{
		DEBUG_INFO(this->get_logger(), "====callback detet====");
		gmps_stamp_ = msg->header.stamp;
		//push queue
		update_mileage(); //mileageの更新
		marker_info blank_info={}; //queue[0]をクリアするため
		marker_queue_[PREV] = marker_queue_[NOW];
		marker_queue_[NOW] = blank_info;
		marker_queue_[NOW].side_diff = msg->side_diff;
		marker_queue_[NOW].pole = msg->pole;
		marker_queue_[NOW].mm_kind = msg->mm_kind;
		marker_queue_[NOW].delay_dist = msg->delay_dist;
		marker_queue_[NOW].yaw_prev_detected = prev_yaw_; //二連マーカの判定のため
		marker_queue_[NOW].mileage = mileage_;
		marker_queue_[NOW].min_i = marker_queue_[PREV].min_i; //min_iは前回値を引き継いでおく。適切なものがあれば上書きされる
		DEBUG_INFO(this->get_logger(), "side_diff=%.3f, pole=%d,delay_dist=%.2f, mileage=%.2f", 
		 marker_queue_[NOW].side_diff, marker_queue_[NOW].pole, marker_queue_[NOW].delay_dist, marker_queue_[NOW].mileage);

		f_marker_association_success_ = false;
		f_valid_rfid_detected_ = false;
		f_double_marker_ = false;
		f_pose_covariance_high_ = (prev_pose_.pose.covariance[6 * 0 + 0] > 1.0) ? true : false; //debug ここでしか登場しないのでメンバ変数にしなくてもよい
		f_ignore_pose_ = (f_pose_covariance_high_ == true) || (f_prev_pose_once_received_ == false); //covarianceが高いかposeを一度も受信していない場合
		DEBUG_INFO(this->get_logger(), "prev_x=%.2f, prev_y=%.2f, yaw=%.5f, cov[0]=%.2f",
		 prev_pose_.pose.pose.position.x, prev_pose_.pose.pose.position.y,prev_yaw_,prev_pose_.pose.covariance[6 * 0 + 0]);
		DEBUG_INFO(this->get_logger(), "f_ignore_pose = %d", f_ignore_pose_);

		/* rfid_tagと距離による磁気マーカの対応付け。tag_id_detectedを更新 */
		if ((param_enable_rfid_==true) && (rfid_queue_.empty() == false))
		{
			DEBUG_INFO(this->get_logger(), "----call association rfid----");
			f_marker_association_success_ = marker_association_by_rfid();
			DEBUG_INFO(this->get_logger(), "----exit association rfid----");
		}
		
		/* RFIDで対応付けができなかった場合はposeによる対応付けをする */
		/* RFIDが無く、covarianceも高い場合は何もできない */
		if ((f_valid_rfid_detected_ == false) && (f_ignore_pose_ == false))
		{
			DEBUG_INFO(this->get_logger(), "----call association pose----");
			/* prev poseによる磁気マーカの対応付け。磁気マーカの埋設位置が特定される。*/
			f_marker_association_success_ = marker_association_by_prev_pose();
			DEBUG_INFO(this->get_logger(), "----exit association pose----");
		}
		
		publish_info(); //対応付けが失敗した場合でもqueueログはpubしておく
		if(f_marker_association_success_ == false)
		{
			RCLCPP_WARN(this->get_logger(), "association fails. this marker was ignored.");
			return; //対応付けに失敗した場合は以降の処理を行わない
		}

		//check if double marker by dist and yaw between PREV and NOW
		if ((marker_queue_[NOW].mm_id > 0) && (marker_queue_[PREV].mm_id > 0)) 
		{ //queueが正常に2個あるときのみ二連マーカの判定をする
			DEBUG_INFO(this->get_logger(), "----call check double marker----");
			f_double_marker_ = check_double_marker();
			DEBUG_INFO(this->get_logger(), "f_double_marker_ = %d", f_double_marker_);
		}

		/* 観測座標の計算 */
		if (f_double_marker_ == true)
		{
			DEBUG_INFO(this->get_logger(), "----call measurement pose by double marker----");
			measurement_double_marker();//二連マーカはX,Y,yaw
			publish_pose();
		}
		else if (f_ignore_pose_ == false) //prev_poseが信頼できないときは単体マーカの処理ができない
		{
			DEBUG_INFO(this->get_logger(), "----call measurement pose by signle marker----");
			measurement_single_marker();//単体マーカはXYのみ
			publish_pose();
		}
		else
		{
			RCLCPP_WARN(this->get_logger(), "cannot calc measurement pose. this marker was ignored.");
		}
		DEBUG_INFO(this->get_logger(), "====exit callback detet====");
	}

	//RFIDキューから今検知したマーカーに対応するものを探してtag_id_detectedに保存する
	bool marker_association_by_rfid()
	{
		/* associate RFID queue */
		DEBUG_INFO(this->get_logger(), "associate RFID queue");
		f_valid_rfid_detected_ = false; //RFIDキューから対応するものを見つけたフラグ
		bool f_tag_id_matched = false; //検知したタグ番号に一致するマーカテーブルを見つけた

		//RFIDを検知してから磁気マーカを検知するまでの走行距離の最大最小。磁気マーカの距離遅延を考慮する。
		min_rfid_assoc_ = param_tf_rfid_x_ - (param_tf_x_ - marker_queue_[NOW].delay_dist) - param_th_rfid_detect_range_m_;
		max_rfid_assoc_ = param_tf_rfid_x_ - (param_tf_x_ - marker_queue_[NOW].delay_dist) + param_th_rfid_detect_range_m_;
		max_rfid_assoc_ = std::max(0.0, max_rfid_assoc_); //オフセットがおかしいとmaxが負になることもありうるため

		const double mileage_gmps = marker_queue_[NOW].mileage;
		DEBUG_INFO(this->get_logger(), "RFID queue size=%ld", rfid_queue_.size());
		/* RFIDのqueueを先頭（最も古い）から距離をチェックしていく */
		while (rfid_queue_.empty() == false)
		{
			const double mileage_rfid = rfid_queue_.front().mileage;
			const double mileage_rfid_gmps = mileage_gmps - mileage_rfid; //RFIDを検知した後にGMPSを検知するはずなので、これは常に正であるべき
			//debug mileageが負の場合がありうる？バグになる？

			if (mileage_rfid_gmps < min_rfid_assoc_)
			{
				DEBUG_INFO(this->get_logger(), "dist < min  ---->  QUIT tag_id=%d", rfid_queue_.front().tag_id_detected);
				break; //最も古いRFIDの走行距離がMIN未満であればこれ以上探索する意味はない
			}
			else if (mileage_rfid_gmps > max_rfid_assoc_)
			{
				DEBUG_INFO(this->get_logger(), "dist > max  ---->  CONTINUE tag_id=%d", rfid_queue_.front().tag_id_detected);
				rfid_queue_.pop(); //走行距離がMAXを超えている場合は破棄する。
				continue; //次のRFIDを調べに行く
			}
			else //min <= dist <= max
			{
				DEBUG_INFO(this->get_logger(), "min < dist < max  ---->  POP!!!");
				marker_queue_[NOW].tag_id_detected = rfid_queue_.front().tag_id_detected;
				rfid_queue_.pop();
				f_valid_rfid_detected_ = true;
				DEBUG_INFO(this->get_logger(), "tag_id_detected=%d corresponding to this marker", marker_queue_[NOW].tag_id_detected)
				break;
			}
		}

		/* 適切なqueueが無ければreturn. 以降の処理を行わない */
		if (f_valid_rfid_detected_ == false)
		{
			DEBUG_INFO(this->get_logger(), "cannot find valid distance RFID queue. continue pose association");
			return false;
		}

		/* associate marker by rfid*/
		const int i_start = marker_queue_[PREV].min_i - BACK_MARGIN; //初回はmin_i=0 基本的には前回値+1のマーカであるはず
		const int i_end = i_start + marker_table_.size(); //設定としては常に全点探索。十分近いものが見つかったら中断する

		DEBUG_INFO(this->get_logger(), "i_start=%d, i_end=%d",i_start, i_end);
		for (int i=i_start; i < i_end; i++)
		{
			int j=i; //indexは本来0~size()-1であるべきところ、ループ対応で範囲を超える場合があるので中間変数を使う
			if (j < 0) //始点を超えたらループ
			{
				j = j + marker_table_.size();
			}
			else if (j >= marker_table_.size()) //終点を超えたらループ
			{
				j = j - marker_table_.size();
			}
			DEBUG_INFO(this->get_logger(), "j=%d", j);

			/* RFID による探索 */
			if(marker_queue_[NOW].tag_id_detected == marker_table_[j].tag_id) //検知したタグ番号と同じ番号が見つかるまで進める
			{
				RCLCPP_INFO(this->get_logger(), "TAG_ID %d matched at table index %d", marker_queue_[NOW].tag_id_detected, j);
				marker_queue_[NOW].min_i = j;
				f_tag_id_matched = true; //note RFIDの対応付けに成功したら、以降の処理で失敗してもprev poseの探索を行わない
				break;
			}
			else
			{
				//DEBUG_INFO(this->get_logger(), "CONTINUE by RFID");
				continue;
			}
		}

		/* マーカーテーブルに一致するタグ番号が見つからなければ終了。以降の処理を行わない */
		if (f_tag_id_matched == false)
		{
			RCLCPP_WARN(this->get_logger(), "cannot find TAG_ID %d in marker table.", marker_queue_[NOW].tag_id_detected);
			return false;
		}

		/* 極性による照合 */
		if ((param_enable_pole_ == true) &&
			(marker_queue_[NOW].pole != marker_table_[marker_queue_[NOW].min_i].pole)) //極性が一致しないときはスキップ
		{
			RCLCPP_WARN(this->get_logger(), "invalid pole. gmps detected=%d, table expected=%d", marker_queue_[NOW].pole, marker_table_[marker_queue_[NOW].min_i].pole);
			return false;
		}

		/* 距離による照合 */
		//prev_poseが信頼できる場合は距離による照合を行う
		if (f_ignore_pose_ == false)
		{
			const double predict_marker_x = prev_pose_.pose.pose.position.x
			 + (param_tf_x_) * cos(prev_yaw_) + (param_tf_y_) * sin(prev_yaw_) //from base_link to gpms_center
			 + (- marker_queue_[NOW].delay_dist) * cos(prev_yaw_ + param_tf_yaw_) + (+ marker_queue_[NOW].side_diff) * sin(prev_yaw_ + param_tf_yaw_); //from gmps_center to marker
			const double predict_marker_y = prev_pose_.pose.pose.position.y
			 + (param_tf_x_) * sin(prev_yaw_) - (param_tf_y_) * cos(prev_yaw_) //from base_link to gmps_center
			 + (- marker_queue_[NOW].delay_dist) * sin(prev_yaw_ + param_tf_yaw_) - (+ marker_queue_[NOW].side_diff) * cos(prev_yaw_ + param_tf_yaw_); //from gmps_center to marker
			
			const double dist = calc_dist(marker_table_[marker_queue_[NOW].min_i].x - predict_marker_x,
			 marker_table_[marker_queue_[NOW].min_i].y - predict_marker_y);

			if (dist > param_th_association_error_dist_m_)
			{
				marker_queue_[NOW].mm_id = -2; //後解析で分かるようにエラー番号を入れておく
				//marker_queue_[NOW].min_i = marker_queue_[PREV].min_i; //対応付けに失敗した場合も次の探索開始地点は前回から引き継ぐ
				RCLCPP_ERROR(this->get_logger(),
				"Ignore detect. dist=%.2f > threshold %.2f", dist, param_th_association_error_dist_m_);
				RCLCPP_ERROR(this->get_logger(),
				"predicted marker pos x=%.2f,y=%.2f is too far from table defined pos x=%.2f,y=%.2f",
				 predict_marker_x, predict_marker_y, marker_table_[marker_queue_[NOW].min_i].x, marker_table_[marker_queue_[NOW].min_i].y);

				return false;
			}
		}

		/* ここまでたどり着いたならOK */
		DEBUG_INFO(this->get_logger(), "return associate rfid");
		marker_queue_[NOW].x_marker = marker_table_[marker_queue_[NOW].min_i].x;
		marker_queue_[NOW].y_marker = marker_table_[marker_queue_[NOW].min_i].y;
		marker_queue_[NOW].mm_id = marker_table_[marker_queue_[NOW].min_i].mm_id;
		marker_queue_[NOW].tag_id_table = marker_table_[marker_queue_[NOW].min_i].tag_id;
		RCLCPP_INFO(this->get_logger(),
		 "push marker queue with mm_id=%d, tag_id=%d, X=%.2lf, Y=%.2lf",
		   marker_queue_[NOW].mm_id, marker_queue_[NOW].tag_id_table,
		    marker_queue_[NOW].x_marker, marker_queue_[NOW].y_marker);
		return true;
		
	}
	
	//自己位置からテーブルを探索する
	bool marker_association_by_prev_pose()
	{
		DEBUG_INFO(this->get_logger(), "association pose called");
        //baselinkからsensor, markerまでの座標変換
			//tfを使うのが望ましいかもだが、一旦よい
		geometry_msgs::msg::Point predict_marker_pose; //自己位置の前回値から予想するマーカー位置
		predict_marker_pose.x = prev_pose_.pose.pose.position.x
		 + (param_tf_x_) * cos(prev_yaw_) + (param_tf_y_) * sin(prev_yaw_) //from base_link to gmps_center
		 + (- marker_queue_[NOW].delay_dist) * cos(prev_yaw_ + param_tf_yaw_) + (+ marker_queue_[NOW].side_diff) * sin(prev_yaw_ + param_tf_yaw_); //from gmps_center to marker
		predict_marker_pose.y = prev_pose_.pose.pose.position.y
		 + (param_tf_x_) * sin(prev_yaw_) - (param_tf_y_) * cos(prev_yaw_) //from base_link to gmps_center
		 + (- marker_queue_[NOW].delay_dist) * sin(prev_yaw_ + param_tf_yaw_) - (+ marker_queue_[NOW].side_diff) * cos(prev_yaw_ + param_tf_yaw_); //from gmps_center to marker
		
		//association
		double min_dist = 100000;
		int min_i = 100000;
		double dist;

		const int i_start = marker_queue_[PREV].min_i - BACK_MARGIN; //初回はmin_i=0
		const int i_end = i_start + marker_table_.size(); //設定としては常に全点探索。十分近いものが見つかったら中断する
#if 0
		if (f_marker_association_success_ == false){ //初回及び前回の探索が失敗したとき //debug 失敗が連続したとき、にすべき？　可読性が低い書き方
			i_end = i_start + marker_table_.size(); 
		}
		else
		{ //基本的には前後数個を探索すれば見つかるはず
			i_end = i_start + 2 * BACK_MARGIN; 
		}
#endif
		DEBUG_INFO(this->get_logger(), "i_start=%d, i_end=%d",i_start, i_end);
		for (int i=i_start; i < i_end; i++)
		{
			int j=i; //indexは本来0~size()-1であるべきところ、ループ対応で範囲を超える場合があるので中間変数を使う
			if (j < 0) //始点を超えたらループ
			{
				j = j + marker_table_.size();
			}
			else if (j >= marker_table_.size()) //終点を超えたらループ
			{
				j = j - marker_table_.size();
			}
			//DEBUG_INFO(this->get_logger(), "j=%d", j);

			/* 極性による探索 */
			if ((param_enable_pole_ == true) &&
				(marker_queue_[NOW].pole != marker_table_[j].pole)) //極性が一致しないときはスキップ
			{
				//DEBUG_INFO(this->get_logger(), "CONTINUE by POLE");
				continue;
			}

			dist = calc_dist(marker_table_[j].x - predict_marker_pose.x, marker_table_[j].y - predict_marker_pose.y);
			// マーカーリストの全点探索
			if (dist < min_dist)
			{
				min_dist = dist;
				min_i = j;

				// かなり近いマーカがあればそこで終了
				if(min_dist < param_th_association_break_dist_m_)
				{
					RCLCPP_INFO(this->get_logger(), "terminate list search by %.2f",param_th_association_break_dist_m_);
					break;
				}
			}
		}

		DEBUG_INFO(this->get_logger(), "check association error dist");
		//　エラー判定　閾値以内のマーカが見つかっていないなら以降の処理は行わない
		if (min_dist > param_th_association_error_dist_m_)
		{
			marker_queue_[NOW].mm_id = -1; //後解析で分かるようにエラー番号を入れておく
				RCLCPP_ERROR(this->get_logger(),
				"Ignore detect. min_dist=%.2f > threshold %.2f", min_dist, param_th_association_error_dist_m_);
				RCLCPP_ERROR(this->get_logger(),
				"predicted marker pos x=%.2f,y=%.2f is too far from table defined pos x=%.2f,y=%.2f",
				 predict_marker_pose.x, predict_marker_pose.y, marker_table_[min_i].x, marker_table_[min_i].y);


			return false;
		}

		//debug 初期位置推定の場合かどうかで処理が大きく分かれる
		DEBUG_INFO(this->get_logger(), "return associate pose");
		// return
		marker_queue_[NOW].min_i = min_i;
		marker_queue_[NOW].x_marker = marker_table_[min_i].x;
		marker_queue_[NOW].y_marker = marker_table_[min_i].y;
		marker_queue_[NOW].mm_id = marker_table_[min_i].mm_id;
		marker_queue_[NOW].tag_id_table = marker_table_[min_i].tag_id;
		RCLCPP_INFO(this->get_logger(),
		 "push marker queue with mm_id=%d, tag_id=%d, X=%.2lf, Y=%.2lf",
		   marker_queue_[NOW].mm_id, marker_queue_[NOW].tag_id_table,
		    marker_queue_[NOW].x_marker, marker_queue_[NOW].y_marker);
		
		return true;
	}

	// 二連マーカとして処理するかどうかの判定
	bool check_double_marker()
	{
		//一個目のマーカから二個目のマーカを検知するまでの走行距離が閾値以内であること
		//debug マーカ埋設座標の静的な距離でいいのでは？
		const double mileage_between_markers = marker_queue_[NOW].mileage - marker_queue_[PREV].mileage;

		//一個目のマーカと二個目のマーカでyawが大きく変化していないこと
		//debug 本当は操舵角等で判断したいが、poseのyawの微分で判断している
		const double yaw_diff_between_markers = 
		 marker_queue_[NOW].yaw_prev_detected - marker_queue_[PREV].yaw_prev_detected;

		return ((0.1 < mileage_between_markers) && //あまりにも近距離で連続して検知した場合は誤検知なので無視する
				(mileage_between_markers < param_th_dist_double_marker_m_) && 
				(abs(yaw_diff_between_markers) < param_th_yaw_diff_double_marker_rad_));
	}

	void measurement_single_marker()
	{
		//markerからsensor, baselinkまでの変換
		measurement_pose_.pose.pose.position.x = marker_queue_[NOW].x_marker 
		 - (- marker_queue_[NOW].delay_dist) * cos(prev_yaw_ + param_tf_yaw_) - (+ marker_queue_[NOW].side_diff) * sin(prev_yaw_ + param_tf_yaw_) //from marker to gmps_center
		 - (param_tf_x_) * cos(prev_yaw_) - (param_tf_y_) * sin(prev_yaw_); //from gmps_center to base_link

		measurement_pose_.pose.pose.position.y = marker_queue_[NOW].y_marker 
		 - (- marker_queue_[NOW].delay_dist) * sin(prev_yaw_ + param_tf_yaw_) + (+ marker_queue_[NOW].side_diff) * cos(prev_yaw_ + param_tf_yaw_) //from marker to gmps_center
		 - (param_tf_x_) * sin(prev_yaw_) + (param_tf_y_) * cos(prev_yaw_); //from gmps_center to base_link

		//z, qはprev_poseから継承
		measurement_pose_.pose.pose.position.z = prev_pose_.pose.pose.position.z;
		measurement_pose_.pose.pose.orientation = prev_pose_.pose.pose.orientation;
		//covariance
		measurement_pose_.pose.covariance[6*0+0] = sigma_x_gmps_ * sigma_x_gmps_; //x*x
		measurement_pose_.pose.covariance[6*1+1] = sigma_y_gmps_ * sigma_y_gmps_; //y*y
		measurement_pose_.pose.covariance[6*5+5] = sigma_theta_gmps_ * sigma_theta_gmps_; //yaw*yaw	
	}

	void measurement_double_marker()
	{
		//磁気マーカ間のyawを計算 1個目[1]を基準とした2個目[0]へのyaw
		const double dx = marker_queue_[NOW].x_marker - marker_queue_[PREV].x_marker;
		const double dy = marker_queue_[NOW].y_marker - marker_queue_[PREV].y_marker;
		const double yaw_marker = atan2(dy, dx);
		//二つの横変位からdthetaを計算
		//マーカ間距離には測量による真値を使用する。prev_poseの比較よりも正しいはず
		const double dist_marker = calc_dist(dx,dy);
		const double dtheta = asin((marker_queue_[NOW].side_diff - marker_queue_[PREV].side_diff) / dist_marker );
		//その合計が現在のyaw
		const double yaw_gmps = yaw_marker + dtheta;

		DEBUG_INFO(this->get_logger(), "PREV x=%.2f,y=%.2f,side=%.3f, NOW x=%.2f,y=%.2f,side=%.3f",
		 marker_queue_[PREV].x_marker, marker_queue_[PREV].y_marker, marker_queue_[PREV].side_diff,
		 marker_queue_[NOW].x_marker, marker_queue_[NOW].y_marker, marker_queue_[NOW].side_diff);
		DEBUG_INFO(this->get_logger(),"yaw_marker=%.5f, dtheta=%.5f, yaw_gmps=%.5f", yaw_marker, dtheta, yaw_gmps);
		
		//markerからsensor, baselinkまでの変換
		measurement_pose_.pose.pose.position.x = marker_queue_[NOW].x_marker 
		 - (- marker_queue_[NOW].delay_dist) * cos(yaw_gmps + param_tf_yaw_) - (+ marker_queue_[NOW].side_diff) * sin(yaw_gmps + param_tf_yaw_) //from marker to gmps_center
		 - (param_tf_x_) * cos(yaw_gmps) - (param_tf_y_) * sin(yaw_gmps); //from gmps_center to base_link
		measurement_pose_.pose.pose.position.y = marker_queue_[NOW].y_marker 
		 - (- marker_queue_[NOW].delay_dist) * sin(yaw_gmps + param_tf_yaw_) + (+ marker_queue_[NOW].side_diff) * cos(yaw_gmps + param_tf_yaw_) //from marker to gmps_center
		 - (param_tf_x_) * sin(yaw_gmps) + (param_tf_y_) * cos(yaw_gmps); //from gmps_center to base_link
		//zはprev_poseから継承
		measurement_pose_.pose.pose.position.z = prev_pose_.pose.pose.position.z;
		//quaternion
		tf2::Quaternion qua;
		qua.setRPY(0,0,yaw_gmps);
		measurement_pose_.pose.pose.orientation.x = qua.x();
		measurement_pose_.pose.pose.orientation.y = qua.y();
		measurement_pose_.pose.pose.orientation.z = qua.z();
		measurement_pose_.pose.pose.orientation.w = qua.w();
		//covariance
		measurement_pose_.pose.covariance[6*0+0] = sigma_x_gmps_ * sigma_x_gmps_; //x*x
		measurement_pose_.pose.covariance[6*1+1] = sigma_y_gmps_ * sigma_y_gmps_; //y*y
		measurement_pose_.pose.covariance[6*5+5] = sigma_theta_gmps_ * sigma_theta_gmps_; //yaw*yaw

		
	}

	//解析用にマーカ番号と横変位が紐づいたデータをpublishする
	void publish_info()
	{
		DEBUG_INFO(this->get_logger(), "publish info");
		gmps_msgs::msg::GmpsInfo info_msg;
		info_msg.header.stamp = gmps_stamp_;
		info_msg.header.frame_id = "gmps";
		//マーカ検知
		info_msg.side_diff = marker_queue_[NOW].side_diff;
		info_msg.pole = marker_queue_[NOW].pole;
		info_msg.mm_kind = marker_queue_[NOW].mm_kind;
		//RFID検知
		info_msg.tag_id_detected = marker_queue_[NOW].tag_id_detected;
		//対応付けが成功した場合のみ
		info_msg.mm_id = marker_queue_[NOW].mm_id;
		info_msg.tag_id_table = marker_queue_[NOW].tag_id_table;
		info_msg.x_marker = marker_queue_[NOW].x_marker;
		info_msg.y_marker = marker_queue_[NOW].y_marker;
		pub_gmps_info_->publish(info_msg);
	}

	void publish_pose()
	{
		DEBUG_INFO(this->get_logger(), "publish /gmps_pose with X=%.2f, Y=%.2f",
		 measurement_pose_.pose.pose.position.x, measurement_pose_.pose.pose.position.y);		
		//note 出力の型を変更する場合があるので中間変数を経由する
		//poseのpublish
		geometry_msgs::msg::PoseWithCovarianceStamped gmps_pose; //output measurement pose for publish
		gmps_pose.header.frame_id = "map"; //frameIDはmap
		gmps_pose.header.stamp = gmps_stamp_; //time stamp は磁気マーカを検知したタイミングなので、detect msgから継承
		gmps_pose.pose.pose.position.x = measurement_pose_.pose.pose.position.x; //X, Yはcallbackで計算
		gmps_pose.pose.pose.position.y = measurement_pose_.pose.pose.position.y;
		gmps_pose.pose.pose.position.z = measurement_pose_.pose.pose.position.z; //Zはprev poseから継承
		gmps_pose.pose.pose.orientation = measurement_pose_.pose.pose.orientation; //qはprev pose または 二連マーカ
		gmps_pose.pose.covariance[6 * 0 + 0] = measurement_pose_.pose.covariance[6*0+0]; //x*x
		gmps_pose.pose.covariance[6 * 1 + 1] = measurement_pose_.pose.covariance[6*1+1]; //y*y
		gmps_pose.pose.covariance[6 * 5 + 5] = measurement_pose_.pose.covariance[6*5+5]; //yaw*yaw
		
		pub_gmps_pose_->publish(gmps_pose);

		geometry_msgs::msg::TransformStamped transformStamped;
		transformStamped.header.stamp = this->now();
		transformStamped.header.frame_id = "map";
		transformStamped.child_frame_id = "gmps_link";
		transformStamped.transform.translation.x = measurement_pose_.pose.pose.position.x;
		transformStamped.transform.translation.y = measurement_pose_.pose.pose.position.y;
		transformStamped.transform.translation.z = measurement_pose_.pose.pose.position.z;

		transformStamped.transform.rotation.x = measurement_pose_.pose.pose.orientation.x;
		transformStamped.transform.rotation.y = measurement_pose_.pose.pose.orientation.y;
		transformStamped.transform.rotation.z = measurement_pose_.pose.pose.orientation.z;
		transformStamped.transform.rotation.w = measurement_pose_.pose.pose.orientation.w;

		tf_br_->sendTransform(transformStamped);
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
			RCLCPP_ERROR(this->get_logger(),"could not open marker_table file %s. please confirm filepath is correct.", csvname);
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
				if(record.mm_id == 0)
				{
					RCLCPP_WARN(this->get_logger(), "marker table include mm_id=0. this may cause unintended consequence" );
				}
			}
		fclose(fp);
	}

	//距離計算するだけ
	double calc_dist(const double dx,const double dy)
	{
		return(sqrt(dx*dx + dy*dy));
	}

	//速度情報をもとに走行距離を更新するだけ
	void update_mileage()
	{
		dt_ = (get_clock()->now() - prev_time_).seconds();
		mileage_ = mileage_ + vx_mps_ * dt_;
		prev_time_ = get_clock()->now();
	}



public:
	GMPSLocalizer(const rclcpp::NodeOptions &node_options)
    	: rclcpp::Node("gmps_localizer", node_options)
	{
		RCLCPP_INFO(this->get_logger(), "constructor start");
        /* parameters */
		this->declare_parameter("enable_pole", true);
		param_enable_pole_ = this->get_parameter("enable_pole").as_bool();
		this->declare_parameter("enable_rfid", false);
		param_enable_rfid_ = this->get_parameter("enable_rfid").as_bool();
		this->declare_parameter("tf_x", 1.0);
		param_tf_x_ = this->get_parameter("tf_x").as_double();
		this->declare_parameter("tf_y", 0.0);
		param_tf_y_ = this->get_parameter("tf_y").as_double();
		this->declare_parameter("tf_yaw", 0.0);
		param_tf_yaw_ = this->get_parameter("tf_yaw").as_double();
		this->declare_parameter("tf_rfid_x", 2.0); //RFIDの
		param_tf_rfid_x_ = this->get_parameter("tf_rfid_x").as_double();
		this->declare_parameter("th_rfid_detect_range_m", 0.5);
		param_th_rfid_detect_range_m_ = this->get_parameter("th_rfid_detect_range_m").as_double();
		this->declare_parameter("th_association_error_dist_m", 0.5);
		param_th_association_error_dist_m_ = this->get_parameter("th_association_error_dist_m").as_double();
		this->declare_parameter("th_association_break_dist_m", 0.3);
		param_th_association_break_dist_m_ = this->get_parameter("th_association_break_dist_m").as_double();
		this->declare_parameter("th_dist_double_marker_m", 2.5);
        param_th_dist_double_marker_m_ = this->get_parameter("th_dist_double_marker_m").as_double();
		this->declare_parameter("th_yaw_diff_double_marker_rad", 0.05);
        param_th_yaw_diff_double_marker_rad_ = this->get_parameter("th_yaw_diff_double_marker_rad").as_double();
        this->declare_parameter("marker_table_csv_name", "gmps_driver/DATA/marker.csv");
        marker_table_csv_name_ = this->get_parameter("marker_table_csv_name").as_string();
		this->declare_parameter("sigma_x_gmps", 0.07);
		sigma_x_gmps_ = this->get_parameter("sigma_x_gmps").as_double();
		this->declare_parameter("sigma_y_gmps", 0.07);
		sigma_y_gmps_ = this->get_parameter("sigma_y_gmps").as_double();
		this->declare_parameter("sigma_theta_gmps", 0.1);
		sigma_theta_gmps_ = this->get_parameter("sigma_theta_gmps").as_double();

        /* Publisher */
        pub_gmps_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("out_gmps_pose", rclcpp::QoS(1));
        pub_gmps_info_ = this->create_publisher<gmps_msgs::msg::GmpsInfo>("out_gmps_info", rclcpp::QoS(1));

        /* Subscriber */
        sub_prev_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "in_prev_pose", rclcpp::QoS(1), std::bind(&GMPSLocalizer::callbackPrevPose, this, std::placeholders::_1));
        sub_gmps_detect_ = this->create_subscription<gmps_msgs::msg::GmpsDetect>(
            "in_gmps_detect", rclcpp::SensorDataQoS(), std::bind(&GMPSLocalizer::callbackDetect, this, std::placeholders::_1));
		sub_rfid_ = this->create_subscription<gmps_msgs::msg::Rfid>(
            "in_rfid", rclcpp::QoS(1), std::bind(&GMPSLocalizer::callbackRfid, this, std::placeholders::_1));
		sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "in_velocity", rclcpp::QoS(1), std::bind(&GMPSLocalizer::callbackVelocity, this, std::placeholders::_1));
		tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    		std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

		/* Timer */
		timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Rate(50).period(), std::bind(&GMPSLocalizer::callbackTimer, this));  //debug こんなしょーもない処理を50Hzでやる必要ある？
		prev_time_ = get_clock()->now();

		//marker.csvのロード
		RCLCPP_INFO(this->get_logger(), "load marker csv");
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
  rclcpp::shutdown();
  return 0;
}
