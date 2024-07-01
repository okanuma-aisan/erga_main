/*
愛知製鋼のEKF
GNSS, NDT, GMPSの３つの観測
*/

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

//#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> //観測は全部これで入ってくる想定
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU> //LU分解？いるんか？

//#include <wada_vmc_msgs/msg/can502_20221111.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
//#include <gmps_msgs/msg/debug_ekf_log.hpp>
#include <std_msgs/msg/float32_multi_array.hpp> //debug_logは気軽にメッセージを増やしたいのでMultiArrayにする

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

class EKFAichiLocalizer : public rclcpp::Node
{
private:
	/* Publisher */
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_ekf_aichi_pose_;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_debug_ekf_log_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_kinematic_stat_;//車両位置と車両速度情報

	/* Subscriber */
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ndt_pose_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gnss_pose_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gmps_pose_;
	rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_velocity_;
	//rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_delta_;
	//rclcpp::Subscription<wada_vmc_msgs::msg::Can502_20221111>::SharedPtr sub_delta_;
	rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_delta_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_gamma_;
	rclcpp::Subscription<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr sub_ndt_tp_;
	rclcpp::Subscription<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr sub_ndt_nvtl_;
	rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gnss_gpgga_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ndt_init_;

	//aichi ekfから車速の取得のやり方が分からなかったので、このkinematic_statで設定する車速はautowareのekfを使用する
	rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_autoware_ekf_twist_;


	/* tf */	
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
	std::string twist_frame_id_;

	/* Timer */
	rclcpp::TimerBase::SharedPtr timer_; //main timer
	std::shared_ptr<const rclcpp::Time> prev_time_; //last time when prediction was called

	/* parameters */
	double timer_freq_; //制御周波数[Hz]
	double dt_; //制御周期 [s] //これは内部で計算するのみ
	double lr_; //重心と後軸中心までの距離 [m]
	double lw_; //ホイールベース[m]
	double Ksf_; //スタビリティファクタ
	double Kbeta0_; //横滑り係数
	double sigma_x_ndt_; //NDT
	double sigma_y_ndt_;
	double sigma_theta_ndt_;
	double sigma_x_gnss_; //GNSS
	double sigma_y_gnss_;
	double sigma_theta_gnss_;
	double sigma_x_gmps_; //GMPSのX座標の標準偏差
	double sigma_y_gmps_; //GMPSのY座標の標準偏差
	double sigma_theta_gmps_; //GMPSのyawの標準偏差 //yawは観測しない。2連マーカの場合は観測する。
	double sigma_v_; //車速の標準偏差
	double sigma_delta_; //タイヤ操舵角の標準偏差
	double sigma_gamma_; //ヨーレートの標準偏差
	double limit_rate_kv_; //レートリミッタ
	double limit_rate_gb_;
	double limit_rate_kb_;
	double min_kv_; //saturation min
	double min_gb_;
	double min_kb_;
	double max_kv_; //saturation max
	double max_gb_;
	double max_kb_;
	double min_ndt_tp_; //transformation probability 高いほど良い
	double min_ndt_nvtl_; //nearest voxel transformation likelihood 高いほど良い
	int min_gnss_fix_quality_; //Fix Quality 機器依存
	double max_gnss_hdop_; //HDOP 低いほど良い
	double max_beta_deg_;
	double max_dist_;
	bool enable_ndt_;
	bool enable_gnss_;
	bool enable_gmps_;
	//double sigma_ndt_init_;    //ndt初期化時にndt位置以外をこの寄与率にして、ndt位置以外の影響をなくす

	/* variables */
	bool f_ndt_update_; //ndt更新フラグ
	bool f_gnss_update_; //gnss更新フラグ
	bool f_gmps_update_; //gmps更新フラグ
	bool f_ndt_ready_; //ndt更新フラグ
	bool f_gnss_ready_; //gnss更新フラグ
	bool f_gmps_ready_; //gmps更新フラグ
	bool f_ndt_init_;     // ndt初期化フラグ

	double ndt_tp_;
	double ndt_nvtl_;
	int gnss_fix_quality_;
	double gnss_hdop_;
	double z_gnss_; //HDOPが良好な場合は、z座標はGNSSの横流し
	double z_ndt_; //HDOPが良好でない場合は、z座標はNDTの横流し
	double z_ekf_aichi_; //カルマンフィルタ利用時のz
	double running_distance_from_last_marker_; //最後の磁気マーカからの走行距離 [m]
	double dist_euclid_ndt_; //debug ログ保存用
	double dist_mahalanobis_ndt_; //debug ログ保存用
	double dist_euclid_gnss_; //debug ログ保存用
	double dist_mahalanobis_gnss_; //debug ログ保存用
	double dist_euclid_gmps_; //debug ログ保存用
	double dist_mahalanobis_gmps_; //debug ログ保存用

	


	/* kalamn filter */
	const uint8_t dim_x_ = 6;
	const uint8_t dim_u_ = 3;
	const uint8_t dim_z_ = 3;
	Eigen::MatrixXd xk_; //state xk = [x, y, theta, kv, gamma_bias, kbeta]
	Eigen::MatrixXd uk_; //input uk = [v, delta, gamma]
	Eigen::MatrixXd zk_; //measurement zk = [x, y, theta]
	Eigen::MatrixXd Pk_; //error covariance matrix Pk
	Eigen::MatrixXd Fk_; //jacobian Fk = df/dx
	Eigen::MatrixXd Bk_; //jacobian Bk = df/du
	Eigen::MatrixXd Hk_; //measurement matrix Hk = dh/dx
	Eigen::MatrixXd Qk_; //process noise Qk
	Eigen::MatrixXd Rk_; //measurement noise Rk

	Eigen::MatrixXd z0_; //NDT = 0
	Eigen::MatrixXd H0_; //
	Eigen::MatrixXd R0_; //
	Eigen::MatrixXd z1_; //GNSS = 1
	Eigen::MatrixXd H1_; //
	Eigen::MatrixXd R1_; //
	Eigen::MatrixXd z2_; //GMPS = 2
	Eigen::MatrixXd H2_; //
	Eigen::MatrixXd R2_; //

	double kv_prev_;
	double gb_prev_;
	double kb_prev_;
	double beta_;



	bool show_debug_info_;
	
	geometry_msgs::msg::TwistWithCovariance autoware_ekf_twist_;//autoware ekfのtwist

	enum IDX {
		X = 0,
		Y = 1,
		THETA = 2,
		KV = 3,
		GB = 4,
		KB = 5,
	};

	enum IDX_U{
		V = 0,
		Delta = 1,
		Gamma = 2,
	};

	void callbackNdtInit(const std_msgs::msg::Bool::SharedPtr init)
	{
		if(init->data == true)
		{
			f_ndt_init_ = true;//ndt初期化開始フラグをON
			kalmanFilterClear();/* kalman filter initialize*/
		}
		else
		{
			f_ndt_init_ = false;//ndt初期化開始フラグをON
		}
	}

	void callbackNdtPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
		check_ndt();
		if ((enable_ndt_ == true) && (f_ndt_ready_ == true) && (f_ndt_init_ == false))
		{
			f_ndt_update_ = true; //poseの受信フラグ。timer側で使った後にfalseにする
		}

		//humbleだとtf2::getYaw関数でリンクエラーが発生するので、yawの取得方法を変更
		z0_(IDX::X) = msg->pose.pose.position.x; //x
		z0_(IDX::Y) = msg->pose.pose.position.y; //y
		tf2::Quaternion qua(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		double roll, pitch, yaw;
		tf2::Matrix3x3(qua).getRPY(roll, pitch, yaw);
		z0_(IDX::THETA) = yaw;//tf2::getYaw(msg->pose.pose.orientation); //yaw

		
		H0_(IDX::X,IDX::X) = 1.0;
		H0_(IDX::Y,IDX::Y) = 1.0;
		H0_(IDX::THETA,IDX::THETA) = 1.0;
		
		#if 1
		R0_(IDX::X,IDX::X) = sigma_x_ndt_ * sigma_x_ndt_; //Covarianceを捨てて固定値でやる
		R0_(IDX::Y,IDX::Y) = sigma_y_ndt_ * sigma_y_ndt_;
		R0_(IDX::THETA,IDX::THETA) = sigma_theta_ndt_ * sigma_theta_ndt_;
		#endif

		#if 0
		R0_(0,0) = msg->pose.covariance.at(0); //Covarianceを真面目に使う
		R0_(0,1) = msg->pose.covariance.at(1);
		R0_(0,2) = msg->pose.covariance.at(5);
		R0_(1,0) = msg->pose.covariance.at(6);
		R0_(1,1) = msg->pose.covariance.at(7);
		R0_(1,2) = msg->pose.covariance.at(11);
		R0_(2,0) = msg->pose.covariance.at(30);
		R0_(2,1) = msg->pose.covariance.at(31);
		R0_(2,2) = msg->pose.covariance.at(35);
		#endif

		z_ndt_ = msg->pose.pose.position.z;
	}

	
	void callbackGnssPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
		check_gnss();
		if ((enable_gnss_ == true) && (f_gnss_ready_ == true) && (f_ndt_init_ == false))
		{
			f_gnss_update_ = true; //poseの受信フラグ。timer側で使った後にfalseにする
		}

		//humbleだとtf2::getYaw関数でリンクエラーが発生するので、yawの取得方法を変更
		z1_(IDX::X) = msg->pose.pose.position.x; //x
		z1_(IDX::Y) = msg->pose.pose.position.y; //y
		tf2::Quaternion qua(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		double roll, pitch, yaw;
		tf2::Matrix3x3(qua).getRPY(roll, pitch, yaw);
		z1_(IDX::THETA) = yaw;//tf2::getYaw(msg->pose.pose.orientation); //yaw
		
		H1_(IDX::X,IDX::X) = 1.0;
		H1_(IDX::Y,IDX::Y) = 1.0;
		H1_(IDX::THETA,IDX::THETA) = 1.0;
		
		#if 1
		R1_(IDX::X,IDX::X) = sigma_x_gnss_ * sigma_x_gnss_; //Covarianceを捨てて固定値でやる
		R1_(IDX::Y,IDX::Y) = sigma_y_gnss_ * sigma_y_gnss_;
		R1_(IDX::THETA,IDX::THETA) = sigma_theta_gnss_ * sigma_theta_gnss_;
		#endif

		#if 0
		R1_(0,0) = msg->pose.covariance.at(0); //Covarianceを真面目に使う
		R1_(0,1) = msg->pose.covariance.at(1);
		R1_(0,2) = msg->pose.covariance.at(5);
		R1_(1,0) = msg->pose.covariance.at(6);
		R1_(1,1) = msg->pose.covariance.at(7);
		R1_(1,2) = msg->pose.covariance.at(11);
		R1_(2,0) = msg->pose.covariance.at(30);
		R1_(2,1) = msg->pose.covariance.at(31);
		R1_(2,2) = msg->pose.covariance.at(35);
		#endif

		z_gnss_ = msg->pose.pose.position.z; //z座標はGNSSの横流し
		
	}

	void callbackGmpsPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
		check_gmps();
		if ((enable_gmps_ == true) && (f_gmps_ready_ == true) && (f_ndt_init_ == false))
		{
			f_gmps_update_ = true; //poseの受信フラグ。timer側で使った後にfalseにする
		}

		//humbleだとtf2::getYaw関数でリンクエラーが発生するので、yawの取得方法を変更
		z2_(IDX::X) = msg->pose.pose.position.x; //x
		z2_(IDX::Y) = msg->pose.pose.position.y; //y
		tf2::Quaternion qua(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		double roll, pitch, yaw;
		tf2::Matrix3x3(qua).getRPY(roll, pitch, yaw);
		z2_(IDX::THETA) = yaw;//tf2::getYaw(msg->pose.pose.orientation); //yaw
		
		H2_(IDX::X,IDX::X) = 1.0;
		H2_(IDX::Y,IDX::Y) = 1.0;
		H2_(IDX::THETA,IDX::THETA) = 1.0;
		
		#if 1
		//gmpsのcovarianceはlocalizer側で固定値を入れているので、そちらを使ってもよいが
		//管理がだるいので、ekf側で再定義する
		R2_(IDX::X,IDX::X) = sigma_x_gmps_ * sigma_x_gmps_; //Covarianceを捨てて固定値でやる
		R2_(IDX::Y,IDX::Y) = sigma_y_gmps_ * sigma_y_gmps_;
		R2_(IDX::THETA,IDX::THETA) = sigma_theta_gmps_ * sigma_theta_gmps_;
		#endif

		#if 0
		R2_(0,0) = msg->pose.covariance.at(0); //Covarianceを真面目に使う
		R2_(0,1) = msg->pose.covariance.at(1);
		R2_(0,2) = msg->pose.covariance.at(5);
		R2_(1,0) = msg->pose.covariance.at(6);
		R2_(1,1) = msg->pose.covariance.at(7);
		R2_(1,2) = msg->pose.covariance.at(11);
		R2_(2,0) = msg->pose.covariance.at(30);
		R2_(2,1) = msg->pose.covariance.at(31);
		R2_(2,2) = msg->pose.covariance.at(35);
		#endif

		running_distance_from_last_marker_ = 0.0; //マーカー踏んだら走行距離をリセット
	}

	void callbackVelocity(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
	{
		uk_(IDX_U::V) = msg->twist.twist.linear.x; //Velocity [m/s]
		Qk_(IDX_U::V, IDX_U::V) = sigma_v_ * sigma_v_;
	}

	//void callbackDelta(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	/*void callbackDelta(const wada_vmc_msgs::msg::Can502_20221111::SharedPtr msg)
	{
		uk_(IDX_U::Delta) = msg->tire_angle_rad;
		Qk_(IDX_U::Delta, IDX_U::Delta) = sigma_delta_ * sigma_delta_;
	}*/
	void callbackDelta(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
	{
		uk_(IDX_U::Delta) = msg->steering_tire_angle;
		Qk_(IDX_U::Delta, IDX_U::Delta) = sigma_delta_ * sigma_delta_;
	}

	void callbackGamma(const sensor_msgs::msg::Imu::SharedPtr msg)
	{
		uk_(IDX_U::Gamma) = -msg->angular_velocity.z; //Gamma, yawrate [rad/s] IMUの向きと逆だったので-1をかける
		Qk_(IDX_U::Gamma, IDX_U::Gamma) = sigma_gamma_ * sigma_gamma_;
	}

	void callbackNdtTp(const tier4_debug_msgs::msg::Float32Stamped::SharedPtr msg)
	{
		ndt_tp_ = msg->data;
	}

	void callbackNdtNvtl(const tier4_debug_msgs::msg::Float32Stamped::SharedPtr msg)
	{
		ndt_nvtl_ = msg->data;
	}

	void callbackGnssGpgga(const nmea_msgs::msg::Gpgga::SharedPtr msg)
	{
		gnss_fix_quality_ = msg->gps_qual;
		gnss_hdop_ = msg->hdop;
	}

	void callbackAutowareEkfTwist(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
	{
		autoware_ekf_twist_ = msg->twist;
	}

	void time_update()
	{
		double x = xk_(IDX::X);
		double y = xk_(IDX::Y);
		double theta = xk_(IDX::THETA);
		double kv = xk_(IDX::KV);
		double gb = xk_(IDX::GB);
		double kb = xk_(IDX::KB);

		double V = uk_(IDX_U::V);
		double delta = uk_(IDX_U::Delta);
		double gamma = uk_(IDX_U::Gamma);

		double k_delta = (Kbeta0_*V*V) / (1 + Ksf_*V*V) * lr_ / lw_; // 後軸中心の場合は分子の+1が消滅する
		beta_ = k_delta * delta;

		beta_ = min(max(beta_, -max_beta_deg_), max_beta_deg_);


		double psi = theta + kb * beta_;
		double dx = V * dt_ * cos(psi);
		double dy = V * dt_ * sin(psi);

		//a priori estimate
		xk_(IDX::X) = x + kv * dx;
		xk_(IDX::Y) = y + kv * dy;
		xk_(IDX::THETA) = theta + (gamma - gb) * dt_;
		xk_(IDX::KV) = kv;
		xk_(IDX::GB) = gb;
		xk_(IDX::KB) = kb;

		//jacobian Fk
		Fk_ = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
		Fk_(IDX::X, IDX::THETA) = -kv*dy;
		Fk_(IDX::X, IDX::KV)    =     dx;
		Fk_(IDX::X, IDX::KB)    = -kv*dy*beta_;
		Fk_(IDX::Y, IDX::THETA) =  kv*dx;
		Fk_(IDX::Y, IDX::KV)    =     dy;
		Fk_(IDX::Y, IDX::KB)    =  kv*dx*beta_;
		Fk_(IDX::THETA, IDX::GB) = -dt_;

		//jacobian Bk
		Bk_ = Eigen::MatrixXd::Zero(dim_x_, dim_u_);
		Bk_(IDX::X, IDX_U::V)         =  kv * dt_ * cos(psi);
		Bk_(IDX::X, IDX_U::Delta)     = -kv * dy * kb * k_delta;
		Bk_(IDX::Y, IDX_U::V)         =  kv * dt_ * sin(psi);
		Bk_(IDX::Y, IDX_U::Delta)     =  kv * dx * kb * k_delta;
		Bk_(IDX::THETA, IDX_U::Gamma) = dt_;

		//a priori estimate error covariance
		Pk_ = Fk_ * Pk_ * Fk_.transpose() + Bk_ * Qk_ * Bk_.transpose();
	}

	void measurement_update(const Eigen::MatrixXd & zk, const Eigen::MatrixXd & Hk, const Eigen::MatrixXd Rk)
	{
		zk_ = zk;
		Hk_ = Hk;
		Rk_ = Rk;

		Eigen::MatrixXd zk_predict; //measurement prediction
		Eigen::MatrixXd Gk; //kalman gain
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_x_, dim_x_); //単位行列

		//measurement prediction (just extract pose from state)
		zk_predict = xk_.block(0,0, dim_z_,1);

		//check yaw
		double check_pi = check_yaw(zk_(IDX::THETA) - zk_predict(IDX::THETA));
		zk_(IDX::THETA) = zk_(IDX::THETA) + check_pi;

		//distance gate
		double dist = calc_mahalanobis_dist(zk_predict, zk, Pk_.block(0, 0, dim_z_, dim_z_) );
		if(dist > max_dist_)
		{
			DEBUG_INFO(this->get_logger(), "ignore this measurement since dist is larger than threshold");
			return;
		}

		//kalman gain
		Gk = Pk_ * Hk_.transpose() * (Hk_ * Pk_ * Hk_.transpose() + Rk_).inverse();

		//a posteriori estimate
		xk_ = xk_ + Gk * (zk_ - zk_predict);

		//a posteriori estimate error covariance
		Pk_ = (I - Gk * Hk_) * Pk_;

	}

	void publish_pose()
	{
		geometry_msgs::msg::PoseWithCovarianceStamped ekf_aichi_pose_; //output pose

		ekf_aichi_pose_.header.frame_id = "map";
		ekf_aichi_pose_.header.stamp = this->now();
		ekf_aichi_pose_.pose.pose.position.x = xk_(IDX::X);
		ekf_aichi_pose_.pose.pose.position.y = xk_(IDX::Y);
		ekf_aichi_pose_.pose.pose.position.z = z_ndt_;//z_gnss_;
		ekf_aichi_pose_.pose.covariance[0] = Pk_(IDX::X, IDX::X);
		ekf_aichi_pose_.pose.covariance[1] = Pk_(IDX::X, IDX::Y);
		ekf_aichi_pose_.pose.covariance[5] = Pk_(IDX::X, IDX::THETA);
		ekf_aichi_pose_.pose.covariance[6] = Pk_(IDX::Y, IDX::X);
		ekf_aichi_pose_.pose.covariance[7] = Pk_(IDX::Y, IDX::Y);
		ekf_aichi_pose_.pose.covariance[11] = Pk_(IDX::Y, IDX::THETA);
		ekf_aichi_pose_.pose.covariance[30] = Pk_(IDX::THETA, IDX::X);
		ekf_aichi_pose_.pose.covariance[31] = Pk_(IDX::THETA, IDX::Y);
		ekf_aichi_pose_.pose.covariance[35] = Pk_(IDX::THETA, IDX::THETA);

		tf2::Quaternion q;
		q.setRPY(0.0, 0.0, xk_(IDX::THETA));
		ekf_aichi_pose_.pose.pose.orientation.x = q.getX();
		ekf_aichi_pose_.pose.pose.orientation.y = q.getY();
		ekf_aichi_pose_.pose.pose.orientation.z = q.getZ();
		ekf_aichi_pose_.pose.pose.orientation.w = q.getW();

		pub_ekf_aichi_pose_->publish(ekf_aichi_pose_);

		geometry_msgs::msg::TransformStamped transformStamped;
		transformStamped.header.stamp = this->now();
		transformStamped.header.frame_id = "map";
		transformStamped.child_frame_id = twist_frame_id_;//"ekf_aichi_link";
		transformStamped.transform.translation.x = ekf_aichi_pose_.pose.pose.position.x;
		transformStamped.transform.translation.y = ekf_aichi_pose_.pose.pose.position.y;
		transformStamped.transform.translation.z = ekf_aichi_pose_.pose.pose.position.z;

		transformStamped.transform.rotation.x = ekf_aichi_pose_.pose.pose.orientation.x;
		transformStamped.transform.rotation.y = ekf_aichi_pose_.pose.pose.orientation.y;
		transformStamped.transform.rotation.z = ekf_aichi_pose_.pose.pose.orientation.z;
		transformStamped.transform.rotation.w = ekf_aichi_pose_.pose.pose.orientation.w;

		tf_br_->sendTransform(transformStamped);
	}

	void publish_kinematic_stat()
	{
				
		nav_msgs::msg::Odometry kinematic_stat;
		kinematic_stat.header.stamp = this->now();
		kinematic_stat.header.frame_id = "map";
		kinematic_stat.child_frame_id = twist_frame_id_;
		kinematic_stat.pose.pose.position.x = xk_(IDX::X);
		kinematic_stat.pose.pose.position.y = xk_(IDX::Y);
		/*if(gnss_hdop_ < max_gnss_hdop_)//HDOPが良好な場合は、GNSSのZを使用
			kinematic_stat.pose.pose.position.z = z_ekf_aichi_ = z_gnss_;
		else//HDOPが良好でない場合は、NDTのZを使用
			kinematic_stat.pose.pose.position.z = z_ekf_aichi_ = z_ndt_;*/
		kinematic_stat.pose.pose.position.z = z_ekf_aichi_ = z_ndt_;
		kinematic_stat.pose.covariance[0] = Pk_(IDX::X, IDX::X);
		kinematic_stat.pose.covariance[1] = Pk_(IDX::X, IDX::Y);
		kinematic_stat.pose.covariance[5] = Pk_(IDX::X, IDX::THETA);
		kinematic_stat.pose.covariance[6] = Pk_(IDX::Y, IDX::X);
		kinematic_stat.pose.covariance[7] = Pk_(IDX::Y, IDX::Y);
		kinematic_stat.pose.covariance[11] = Pk_(IDX::Y, IDX::THETA);
		kinematic_stat.pose.covariance[30] = Pk_(IDX::THETA, IDX::X);
		kinematic_stat.pose.covariance[31] = Pk_(IDX::THETA, IDX::Y);
		kinematic_stat.pose.covariance[35] = Pk_(IDX::THETA, IDX::THETA);
		kinematic_stat.twist = autoware_ekf_twist_;
		tf2::Quaternion q;
		q.setRPY(0.0, 0.0, xk_(IDX::THETA));
		kinematic_stat.pose.pose.orientation.x = q.getX();
		kinematic_stat.pose.pose.orientation.y = q.getY();
		kinematic_stat.pose.pose.orientation.z = q.getZ();
		kinematic_stat.pose.pose.orientation.w = q.getW();
		pub_kinematic_stat_->publish(kinematic_stat);
	}

	void publish_debug_log()
	{
		std_msgs::msg::Float32MultiArray log_msg;
		log_msg.data.size(38);
		log_msg[0] = z0_(0);//ndt x
		log_msg[1] = z0_(1);//ndt y
		log_msg[2] = z_ndt_;//ndt z
		log_msg[3] = z0_(2);//ndt yaw
		log_msg[4] = z1_(0);//gnss x 
		log_msg[5] = z1_(1);//gnss y
		log_msg[6] = z_gnss_;//gnss z
		log_msg[7] = z1_(2);//gnss yaw
		log_msg[8] = z2_(0);//gmps x
		log_msg[9] = z2_(1);//gmps y
		log_msg[10] = z2_(2);//gmps yaw
		log_msg[11] = xk_(IDX::X);//ekf x
		log_msg[12] = xk_(IDX::Y);//ekf y
		log_msg[13] = z_ekf_aichi_;//ekf z
		log_msg[14] = xk_(IDX::THETA);//ekf yaw
		log_msg[15] = xk_(IDX::KV);//ekf kv
		log_msg[16] = xk_(IDX::GB);//ekf gb
		log_msg[17] = xk_(IDX::KB);//ekf kb
		log_msg[18] = beta_;//sideslip angle
		log_msg[19] = uk_(IDX_U::V);//in_vx_raw
		log_msg[20] = uk_(IDX_U::Delta);//in_delta_raw
		log_msg[21] = uk_(IDX_U::Gamma);//in_gamma_raw
		log_msg[22] = f_ndt_update_;
		log_msg[23] = f_gnss_update_;
		log_msg[24] = f_gmps_update_;
		log_msg[25] = f_ndt_ready_;
		log_msg[26] = f_gnss_ready_;
		log_msg[27] = f_gmps_ready_;
		log_msg[28] = ndt_tp_;
		log_msg[29] = ndt_nvtl_;
		log_msg[30] = gnss_fix_quality_;
		log_msg[31] = gnss_hdop_;
		log_msg[32] = dist_euclid_ndt_;
		log_msg[33] = dist_euclid_gnss_;
		log_msg[34] = dist_euclid_gmps_;
		log_msg[35] = dist_mahalanobis_ndt_;
		log_msg[36] = dist_mahalanobis_gnss_;
		log_msg[37] = dist_mahalanobis_gmps_;


		pub_debug_ekf_log_->publish(log_msg);

	}

	void callbackTimer()
	{
		//DEBUG_INFO(this->get_logger(), "timer called");
		/* update dt */
		dt_ = (get_clock()->now() - *prev_time_).seconds();
		prev_time_ = std::make_shared<const rclcpp::Time>(get_clock()->now());

		/* time update */
		//DEBUG_INFO(this->get_logger(), "time update start");
		time_update();
		if (running_distance_from_last_marker_ <100.0)
		{
			running_distance_from_last_marker_ = running_distance_from_last_marker_ + uk_(IDX_U::V) * dt_;
		}
		//DEBUG_INFO(this->get_logger(), "time update end");
		
		//debug ログ保存用にユークリッド距離とマハラノビス距離を計算する
		//debug ここのPkはSk=Hk*Pk*Hk'+Rkであるべきなのでは？といつも思う
		Eigen::Vector3d zk_predict(xk_(IDX::X), xk_(IDX::Y), xk_(IDX::THETA));
		Eigen::Matrix3d P = Pk_.block(0,0, dim_z_, dim_z_);
		Eigen::Matrix3d I = Eigen::MatrixXd::Identity(dim_z_,dim_z_);
		Eigen::Vector3d zk_ndt(z0_(0), z0_(1), z0_(2));
		Eigen::Vector3d zk_gnss(z1_(0), z1_(1), z1_(2));
		Eigen::Vector3d zk_gmps(z2_(0), z2_(1), z2_(2));
		dist_euclid_ndt_ = calc_mahalanobis_dist(zk_predict, zk_ndt, I);
		dist_euclid_gnss_ = calc_mahalanobis_dist(zk_predict, zk_gnss, I);
		dist_euclid_gmps_ = calc_mahalanobis_dist(zk_predict, zk_gmps, I);
		dist_mahalanobis_ndt_ = calc_mahalanobis_dist(zk_predict, zk_ndt, P);
		dist_mahalanobis_gnss_ = calc_mahalanobis_dist(zk_predict, zk_gnss, P);
		dist_mahalanobis_gmps_ = calc_mahalanobis_dist(zk_predict, zk_gmps, P);


		/* measurement update */
		//そのときに受信したposeを現在時刻の観測として用いる
		if (f_ndt_update_ == true)
		{
			//DEBUG_INFO(this->get_logger(), "measurement update ndt start");
			measurement_update(z0_, H0_, R0_);
			//DEBUG_INFO(this->get_logger(), "measurement update ndt end");
		}

		if (f_gnss_update_ == true)
		{
			//DEBUG_INFO(this->get_logger(), "measurement update gnss start");
			measurement_update(z1_, H1_, R1_);
			//DEBUG_INFO(this->get_logger(), "measurement update gnss end");
		}

		if (f_gmps_update_ == true)
		{
			//DEBUG_INFO(this->get_logger(), "measurement update gmps start");
			measurement_update(z2_, H2_, R2_);
			//DEBUG_INFO(this->get_logger(), "measurement update gmps end");
		}

		/* check yaw */
		double check_pi = check_yaw(xk_(IDX::THETA));
		xk_(IDX::THETA) = xk_(IDX::THETA) + check_pi;

		/* saturation */
		xk_(IDX::KV) = min(max(xk_(IDX::KV), min_kv_), max_kv_);
		xk_(IDX::GB) = min(max(xk_(IDX::GB), min_gb_), max_gb_);
		xk_(IDX::KB) = min(max(xk_(IDX::KB), min_kb_), max_kb_);

		/* rate limitter */
		xk_(IDX::KV) = min(max(xk_(IDX::KV), kv_prev_ - limit_rate_kv_*dt_), kv_prev_ + limit_rate_kv_*dt_);
		xk_(IDX::GB) = min(max(xk_(IDX::GB), gb_prev_ - limit_rate_gb_*dt_), gb_prev_ + limit_rate_gb_*dt_);
		xk_(IDX::KB) = min(max(xk_(IDX::KB), kb_prev_ - limit_rate_kb_*dt_), kb_prev_ + limit_rate_kb_*dt_);
		kv_prev_ = xk_(IDX::KV);
		gb_prev_ = xk_(IDX::GB);
		kb_prev_ = xk_(IDX::KB);

		/* publish */
		publish_pose();
		//DEBUG_INFO(this->get_logger(), "timer end");
		publish_debug_log();

		f_ndt_update_ = false; //logをpublishしてからフラグを落とす
		f_gnss_update_ = false;
		f_gmps_update_ = false;

		//kinematic statのpublish
		publish_kinematic_stat();
	}

	void kalmanFilterClear()
	{
		/* kalman filter initialize*/
		xk_ = Eigen::MatrixXd::Zero(dim_x_, 1); // kv, kbは初期値が１
		xk_(IDX::KV) = 1.0;
		xk_(IDX::GB) = 0.0;
		xk_(IDX::KB) = 1.0;
		kv_prev_ = 1.0;
		gb_prev_ = 0.0;
		kb_prev_ = 1.0;

		Pk_ = Eigen::MatrixXd::Zero(dim_x_, dim_x_); // Pkの初期値は適当
		Pk_(IDX::X, IDX::X) = 1.0E15;
		Pk_(IDX::Y, IDX::Y) = 1.0E15;
		Pk_(IDX::THETA, IDX::THETA) = 50.0;
		Pk_(IDX::KV, IDX::KV) = 0.01;
		Pk_(IDX::GB, IDX::GB) = 0.01;
		Pk_(IDX::KB, IDX::KB) = 0.01;

		uk_ = Eigen::MatrixXd::Zero(dim_u_, 1); // 後で上書きする前提なので、zeroで初期化だけ行う
		Qk_ = Eigen::MatrixXd::Zero(dim_u_, dim_u_);
		z0_ = Eigen::MatrixXd::Zero(dim_z_, 1);
		H0_ = Eigen::MatrixXd::Zero(dim_z_, dim_x_); // Hは変わらない。
		R0_ = Eigen::MatrixXd::Zero(dim_z_, dim_z_);
		z1_ = Eigen::MatrixXd::Zero(dim_z_, 1);
		H1_ = Eigen::MatrixXd::Zero(dim_z_, dim_x_); // Hは変わらない。
		R1_ = Eigen::MatrixXd::Zero(dim_z_, dim_z_);
		z2_ = Eigen::MatrixXd::Zero(dim_z_, 1);
		H2_ = Eigen::MatrixXd::Zero(dim_z_, dim_x_); // Hは変わらない。
		R2_ = Eigen::MatrixXd::Zero(dim_z_, dim_z_);

		running_distance_from_last_marker_ = 100.0;

		z_ndt_ = z_gnss_ = 0;
	}

	double check_yaw(const double &yaw)
	{
		double ret_pi = 0;

		if (yaw > M_PI)
		{
			ret_pi = -2.0*M_PI;
		}
		if (yaw < -M_PI)
		{
			ret_pi = +2.0*M_PI;
		}

		return ret_pi;
	}

	void check_ndt()
	{
		if ((ndt_tp_ > min_ndt_tp_) &&
		    (ndt_nvtl_ > min_ndt_nvtl_) &&
			(running_distance_from_last_marker_ > 11.0))
		{
			f_ndt_ready_ = true;
		}
		else
		{
			f_ndt_ready_ = false;
		}
	}

	void check_gnss()
	{
		if ((gnss_fix_quality_ > min_gnss_fix_quality_) &&
		    (gnss_hdop_ < max_gnss_hdop_) &&
			(running_distance_from_last_marker_ > 11.0))
		{
			f_gnss_ready_ = true;
		}
		else
		{
			f_gnss_ready_ = false;
		}
	}

	void check_gmps()
	{
		if (true)
		{
			f_gmps_ready_ = true;
		}
		else
		{
			f_gmps_ready_ = false;
		}
	}

	double calc_mahalanobis_dist(const Eigen::VectorXd & zk_predict, const Eigen::VectorXd & zk_measurement, const Eigen::MatrixXd & Pk)
	{
		const Eigen::VectorXd d = zk_measurement - zk_predict;
		const double dist_square = d.dot(Pk.inverse() * d);
		return (sqrt(dist_square));
	}


public:
	EKFAichiLocalizer(const rclcpp::NodeOptions &node_options)
		: rclcpp::Node("ekf_aichi_localizer", node_options)
		, f_ndt_init_(false)
	{
		//DEBUG_INFO(this->get_logger(), "constructor");
		/* parameters */
		twist_frame_id_ = this->declare_parameter("twist_frame_id", "ekf_aichi_link");
		timer_freq_ = this->declare_parameter("timer_freq", 50.0);
		dt_ = 1.0 / timer_freq_; //ここの値はtimerの生成のときに使うだけで、以降は上書きされる

		this->declare_parameter<double>("lr", 1.5);
		this->declare_parameter<double>("lw", 4.0);
		this->declare_parameter<double>("Ksf", 0.001);
		this->declare_parameter<double>("Kbeta0", -0.001);
		lr_ = this->get_parameter("lr").as_double();
		lw_ = this->get_parameter("lw").as_double();
		Ksf_ = this->get_parameter("Ksf").as_double();
		Kbeta0_ = this->get_parameter("Kbeta0").as_double();

		this->declare_parameter<double>("sigma_x_ndt", 0.07);
		this->declare_parameter<double>("sigma_y_ndt", 0.07);
		this->declare_parameter<double>("sigma_theta_ndt", 0.1);
		this->declare_parameter<double>("sigma_x_gnss", 0.07);
		this->declare_parameter<double>("sigma_y_gnss", 0.07);
		this->declare_parameter<double>("sigma_theta_gnss", 0.1);
		this->declare_parameter<double>("sigma_x_gmps", 0.07);
		this->declare_parameter<double>("sigma_y_gmps", 0.07);
		this->declare_parameter<double>("sigma_theta_gmps", 0.1);
		this->declare_parameter<double>("sigma_v", 0.03);
		this->declare_parameter<double>("sigma_delta", 0.04);
		this->declare_parameter<double>("sigma_gamma", 0.005);
		sigma_x_ndt_ = this->get_parameter("sigma_x_ndt").as_double();
		sigma_y_ndt_ = this->get_parameter("sigma_y_ndt").as_double();
		sigma_theta_ndt_ = this->get_parameter("sigma_theta_ndt").as_double();
		sigma_x_gnss_ = this->get_parameter("sigma_x_gnss").as_double();
		sigma_y_gnss_ = this->get_parameter("sigma_y_gnss").as_double();
		sigma_theta_gnss_ = this->get_parameter("sigma_theta_gnss").as_double();
		sigma_x_gmps_ = this->get_parameter("sigma_x_gmps").as_double();
		sigma_y_gmps_ = this->get_parameter("sigma_y_gmps").as_double();
		sigma_theta_gmps_ = this->get_parameter("sigma_theta_gmps").as_double();
		sigma_v_ = this->get_parameter("sigma_v").as_double();
		sigma_delta_ = this->get_parameter("sigma_delta").as_double();
		sigma_gamma_ = this->get_parameter("sigma_gamma").as_double();

		this->declare_parameter<double>("limit_rate_kv", 0.001);
		this->declare_parameter<double>("limit_rate_gb", 0.0001);
		this->declare_parameter<double>("limit_rate_kb", 0.001);
		this->declare_parameter<double>("min_kv", 0.9);
		this->declare_parameter<double>("max_kv", 1.1);
		this->declare_parameter<double>("min_gb", -0.001);
		this->declare_parameter<double>("max_gb", +0.001);
		this->declare_parameter<double>("min_kb", 0.9);
		this->declare_parameter<double>("max_kb", 1.1);
		limit_rate_kv_ = this->get_parameter("limit_rate_kv").as_double();
		limit_rate_gb_ = this->get_parameter("limit_rate_gb").as_double();
		limit_rate_kb_ = this->get_parameter("limit_rate_kb").as_double();
		min_kv_ = this->get_parameter("min_kv").as_double();
		max_kv_ = this->get_parameter("max_kv").as_double();
		min_gb_ = this->get_parameter("min_gb").as_double();
		max_gb_ = this->get_parameter("max_gb").as_double();
		min_kb_ = this->get_parameter("min_kb").as_double();
		max_kb_ = this->get_parameter("max_kb").as_double();

		this->declare_parameter<double>("min_ndt_tp", 3.0);
		this->declare_parameter<double>("min_ndt_nvtl", 1.5);
		this->declare_parameter<int>("min_gnss_fix_quality", 3);
		this->declare_parameter<double>("max_gnss_hdop", 2.0);
		this->declare_parameter<double>("max_beta_deg", 3.0);
		this->declare_parameter<double>("max_dist", 0.5);
		this->declare_parameter<bool>("enable_ndt", true);
		this->declare_parameter<bool>("enable_gnss", true);
		this->declare_parameter<bool>("enable_gmps", true);
		min_ndt_tp_ = this->get_parameter("min_ndt_tp").as_double();
		min_ndt_nvtl_= this->get_parameter("min_ndt_nvtl").as_double();
		min_gnss_fix_quality_= this->get_parameter("min_gnss_fix_quality").as_int();
		max_gnss_hdop_ = this->get_parameter("max_gnss_hdop").as_double();
		max_beta_deg_ = this->get_parameter("max_beta_deg").as_double();
		max_dist_ = this->get_parameter("max_dist").as_double();
		enable_ndt_ = this->get_parameter("enable_ndt").as_bool();
		enable_gnss_ = this->get_parameter("enable_gnss").as_bool();
		enable_gmps_ = this->get_parameter("enable_gmps").as_bool();


		show_debug_info_ = this->declare_parameter("show_debug_info", false);

		
		/* timer */
		auto period_control_ns =
		std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt_));
		timer_ = rclcpp::create_timer(
			this, get_clock(), period_control_ns, std::bind(&EKFAichiLocalizer::callbackTimer, this));
		prev_time_ = std::make_shared<const rclcpp::Time>(get_clock()->now());


		/* Publisher */
		pub_ekf_aichi_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("out_ekf_aichi_pose", rclcpp::QoS(1));
		pub_debug_ekf_log_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("debug_ekf_log", rclcpp::QoS(1));
		pub_kinematic_stat_ = this->create_publisher<nav_msgs::msg::Odometry>("out_odom_name", rclcpp::QoS(1));

		/* Subscriber */
		sub_ndt_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		 "in_ndt_pose", rclcpp::SensorDataQoS(), std::bind(&EKFAichiLocalizer::callbackNdtPose, this, std::placeholders::_1));
		sub_gnss_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		 "in_gnss_pose", rclcpp::SensorDataQoS(), std::bind(&EKFAichiLocalizer::callbackGnssPose, this, std::placeholders::_1));
		sub_gmps_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		 "in_gmps_pose", rclcpp::SensorDataQoS(), std::bind(&EKFAichiLocalizer::callbackGmpsPose, this, std::placeholders::_1));
		//sub_velocity_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		// "in_velocity", rclcpp::SensorDataQoS(), std::bind(&EKFAichiLocalizer::callbackVelocity, this, std::placeholders::_1));
		sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
		 "in_velocity", rclcpp::SensorDataQoS(), std::bind(&EKFAichiLocalizer::callbackVelocity, this, std::placeholders::_1));
		//sub_delta_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		// "in_delta", rclcpp::SensorDataQoS(), std::bind(&EKFAichiLocalizer::callbackDelta, this, std::placeholders::_1));
		//sub_delta_ = this->create_subscription<wada_vmc_msgs::msg::Can502_20221111>(
		//	"in_delta", rclcpp::SensorDataQoS(), std::bind(&EKFAichiLocalizer::callbackDelta, this, std::placeholders::_1));
		sub_delta_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
			"in_delta", rclcpp::SensorDataQoS(), std::bind(&EKFAichiLocalizer::callbackDelta, this, std::placeholders::_1));
		sub_gamma_ = this->create_subscription<sensor_msgs::msg::Imu>(
			"in_gamma", rclcpp::SensorDataQoS(), std::bind(&EKFAichiLocalizer::callbackGamma, this, std::placeholders::_1));
		sub_ndt_tp_ = this->create_subscription<tier4_debug_msgs::msg::Float32Stamped>(
			"in_ndt_tp", 10, std::bind(&EKFAichiLocalizer::callbackNdtTp, this, std::placeholders::_1));
		sub_ndt_nvtl_ = this->create_subscription<tier4_debug_msgs::msg::Float32Stamped>(
			"in_ndt_nvtl", 10, std::bind(&EKFAichiLocalizer::callbackNdtNvtl, this, std::placeholders::_1));
		sub_gnss_gpgga_ = this->create_subscription<nmea_msgs::msg::Gpgga>(
			"in_gnss_gpgga", 10, std::bind(&EKFAichiLocalizer::callbackGnssGpgga, this, std::placeholders::_1));
		sub_autoware_ekf_twist_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
			"in_autoware_ekf_twist", 10, std::bind(&EKFAichiLocalizer::callbackAutowareEkfTwist, this, std::placeholders::_1));
		sub_ndt_init_ = this->create_subscription<std_msgs::msg::Bool>(
			"ndt_init", 10, std::bind(&EKFAichiLocalizer::callbackNdtInit, this, std::placeholders::_1));
		tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
			std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

		/* kalman filter initialize*/
		kalmanFilterClear();
	}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<EKFAichiLocalizer>(node_options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
