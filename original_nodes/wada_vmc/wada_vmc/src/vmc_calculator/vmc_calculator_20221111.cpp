// ros2 header
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

// joy board msg
#include <wada_vmc_msgs/msg/can100_20221111.hpp>
#include <wada_vmc_msgs/msg/steer_cmd.hpp>

// autoware msgs
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <auto_permission_msgs/msg/auto_permission.hpp>

// joy board srv
#include <wada_vmc_msgs/srv/actual_input_unsigned.hpp> //車両操作
#include <wada_vmc_msgs/srv/actual_input_float.hpp>   //車両操作
#include <wada_vmc_msgs/srv/do_input.hpp>			   //DO指令

// その他header
#include <wada_vmc/joyboard_receive_info20221111.hpp>
#include <wada_vmc/joyboard_send_info20221111.hpp>
#include <wada_vmc/car_info20221111.hpp>

namespace wada_vmc
{
	class VmcCalculator20221111 : public rclcpp::Node
	{
	private:// 定数
		constexpr static int TIMER_MESSAGE100_HZ = 100;			 // timer_message100_タイマーの周期
		constexpr static double TIMER_BLINKER_NEUTRAL_SEC = 0.2; // ウィンカー通達指令時間

	private:// joyボード送信用publisher
		rclcpp::Publisher<wada_vmc_msgs::msg::Can100_20221111>::SharedPtr pub_can100_; // joyボード受信データ0x100番のpublisher
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_handle_auto_permission_; // ハンドル自動可否
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_pedal_auto_permission_;  // ペダル自動可否

	private:// autowareへのpublisher
		rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr pub_control_mode_; // AUTOWAREコントロールモード
		rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_tmp_int16_;

	private:// joyボード受信用subscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::Can501_20221111>::SharedPtr sub_can501_; // joyボード受信データ0x501番のsubscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::Can502_20221111>::SharedPtr sub_can502_; // joyボード受信データ0x502番のsubscriber
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pedal_clutch_latch_;	  // ブレーキペダル保持情報(contec側がfalseのとき、can側をtrue)

	private:// autowareからのsubscriber
		rclcpp::Subscription<wada_vmc_msgs::msg::SteerCmd>::SharedPtr sub_steer_cmd_; // ステアコマンド値
		rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr sub_actuation_cmd_;//autowareの加速度MAPノードから送られるjoyボードコマンド値
		//rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr sub_autoware_state_;//autowareからの自動運転許可フラグ
		rclcpp::Subscription<auto_permission_msgs::msg::AutoPermission>::SharedPtr sub_auto_permission_;//自動運転許可フラグ

	private:// joyボード制御用サービス
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_steer_auto_;					       // ステアの自動切り替え
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_pedal_auto_;					       // ペダルの自動切り替え
		rclcpp::Service<wada_vmc_msgs::srv::ActualInputUnsigned>::SharedPtr srv_steer_input_;      // ステアの手動入力操作用(joyボードの値をそのまま入力)
		rclcpp::Service<wada_vmc_msgs::srv::ActualInputFloat>::SharedPtr srv_steer_input_tier_;   // ステアの手動入力操作用(タイヤ角を入力)
		rclcpp::Service<wada_vmc_msgs::srv::ActualInputUnsigned>::SharedPtr srv_pedal_input_;      // ペダルの手動入力入力)
		rclcpp::Service<wada_vmc_msgs::srv::DOInput>::SharedPtr srv_steer_do_;				       // ステア側DO指令
		rclcpp::Service<wada_vmc_msgs::srv::DOInput>::SharedPtr srv_pedal_do_;				       // ペダル側DO指令
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_steer_auto_permission_;		       // ステア自動可否
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_pedal_auto_permission_;		       // ペダル自動可否
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_steer_failsafe_;				       // ステアフェイルセーフ
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_pedal_failsafe_;				       // ペダルフェイルセーフ
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_brake_takeover_;				       // ブレーキテイクオーバー
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_steer_clutch_ban_;			       // ステアクラッチ切断指令(ステアDO指令1bit)
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_pedal_clutch_ban_;			       // ペダルクラッチ切断指令(ペダルDO指令1bit)
		rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_autoware_input_mode_;			       // 車両操作を手動入力操作からAUTOWARE操作にする
		// rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_blinker_input_mode_;//ウィンカー操作を手動入力操作からAUTOWARE操作にする

	private:// ros2 timer
		rclcpp::TimerBase::SharedPtr timer_; // 通常処理用タイマー

	private:// 各種情報
		JoyBoardReceiveInfo joy_receive_info_; // joyボード受信情報
		JoyBoardSendInfo joy_send_info_;	   // joyボード送信情報
		CarInfo car_info_;					   // 車両固有情報
		rclcpp::Duration life_cycle_;		   // データの生存時間
		int16_t prev_steer_cmd_;

		double tire_rad_to_steering_can_value_left_slope_;//タイヤ角からステアのCAN指令に変換する係数(左傾き)
		double tire_rad_to_steering_can_value_right_slope_;//タイヤ角からステアのCAN指令に変換する係数(右傾き)
		double tire_rad_to_steering_can_value_left_intercept_;//タイヤ角からステアのCAN指令に変換する係数(左切片)
		double tire_rad_to_steering_can_value_right_intercept_;//タイヤ角からステアのCAN指令に変換する係数(右切片)
		bool pedal_clutch_change_;//!< ペダルクラッチのON→OFF情報
		bool steer_clutch_change_;//!< ステアクラッチのON→OFF情報

	private: // joyボード受信用callback
		// joyボード受信データ0x501のcallback
		//[in] message501:ID501番の情報
		void callbackCan501(const wada_vmc_msgs::msg::Can501_20221111::SharedPtr message501)
		{
			joy_receive_info_.add501(*message501, this->now());
			//!< okanuma ペダルのクラッチが切れた際に一緒にステアのクラッチを切るよう機能追加
			if(joy_receive_info_.get501Size() >= 2)	//!< サイズ2以下はチェックなし
			{
				//!< クラッチがONからOFFに切り替わった時のみ確認
				if(joy_receive_info_.get501(1).clutch == true && joy_receive_info_.get501(0).clutch == false)
				{
					pedal_clutch_change_ = true;
				}
				else
				{
					pedal_clutch_change_ = false;
				}
			}
		}

		// joyボード受信データ0x502のcallback
		//[in] message502:ID502番の情報
		void callbackCan502(const wada_vmc_msgs::msg::Can502_20221111::SharedPtr message502)
		{
			joy_receive_info_.add502(*message502, this->now());
			//!< okanuma ステアのクラッチが切れた際に一緒にステアのクラッチを切るよう機能追加
			if(joy_receive_info_.get502Size() >= 2)	//!< サイズ2以下はチェックなし
			{
				//!< クラッチがONからOFFに切り替わった時のみ確認
				if(joy_receive_info_.get502(1).clutch == true && joy_receive_info_.get502(0).clutch == false)
				{
					steer_clutch_change_ = true;
				}
				else
				{
					steer_clutch_change_ = false;
				}
			}
		}

	private: // autowareからのcallback
		// ステアコマンド値のcallback
		//[in] cmd:ステアコマンド値
		void callbackSteerCmd(const wada_vmc_msgs::msg::SteerCmd::SharedPtr cmd)
		{
			joy_send_info_.addSteerCommand(cmd->steer_cmd, this->now());
		}

		//AUTOWAREからの加速度MAPから作成したペダルコマンド値
		//[in] cmd:ペダルコマンド値
		void callbackActuationCmd(const tier4_vehicle_msgs::msg::ActuationCommandStamped::SharedPtr cmd)
		{
			double brake = cmd->actuation.brake_cmd - 1024;
			brake *= car_info_.cmd_brake_scale_factor_;
			brake += 1024;
			double pedal_cmd = (cmd->actuation.accel_cmd == 0.0) ? brake+0.5 : cmd->actuation.accel_cmd-0.5;
			joy_send_info_.addPedalCommand(static_cast<int16_t>(pedal_cmd), this->now());
		}

		//AUTOWAREからの自動運転可否フラグ
		//[in] autoware_stat:許可フラグ
		/*void callbackAutowareState(const autoware_adapi_v1_msgs::msg::OperationModeState autoware_state)
		{
			if (autoware_state.mode != autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS)
			{
				joy_send_info_.setSteerAutoPermission(false);
				joy_send_info_.setPedalAutoPermission(false);
				joy_send_info_.setSteerFailsafe(true);
				joy_send_info_.setSteerFailsafe(true);
			}
			else
			{
				joy_send_info_.setSteerAutoPermission(true);
				joy_send_info_.setPedalAutoPermission(true);
				joy_send_info_.setSteerFailsafe(false);
				joy_send_info_.setSteerFailsafe(false);
			}
		}*/

		//ブレーキペダル保持情報(contec側がfalseのとき、can側をtrue)
		void callbackPedalClutchLatch(const std_msgs::msg::Bool::ConstSharedPtr msg)
		{
			joy_send_info_.setPedalClutchLatch(msg->data);
		}

		//自動運転許可フラグ
		void callbackAutoPermission(const auto_permission_msgs::msg::AutoPermission::ConstSharedPtr perm)
		{
			if(perm->pedal_permission == auto_permission_msgs::msg::AutoPermission::PERM_OK && perm->steer_permission == auto_permission_msgs::msg::AutoPermission::PERM_OK)
			{
				if(steer_clutch_change_ || pedal_clutch_change_)//!< どちらかのクラッチがON→OFFになったとき両方のクラッチを切断する
				{
					joy_send_info_.setSteerAutoPermission(false);
					joy_send_info_.setPedalAutoPermission(false);
					joy_send_info_.setSteerFailsafe(true);
					joy_send_info_.setPedalFailsafe(true);
				}
				else
				{
					joy_send_info_.setPedalAutoPermission(true);
					joy_send_info_.setSteerAutoPermission(true);
				}
			}
			else
			{
				joy_send_info_.setSteerAutoPermission(false);
				joy_send_info_.setPedalAutoPermission(false);
				joy_send_info_.setSteerFailsafe(true);
				joy_send_info_.setPedalFailsafe(true);
			}

		}

	private: // joyボード制御用サービス
		// ステアの自動切り替え
		//[in] req:ステア自動切り替え指令
		//[out] res:変更内容をそのまま返している
		void serviceSteerAuto(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			joy_send_info_.setSteerAuto(req->data);
			res->success = joy_send_info_.getSteerAuto();
			res->message = "";
		}

		// ペダルの自動切り替え
		//[in] req:ペダル自動切り替え指令
		//[out] res:変更内容をそのまま返している
		void servicePedalAuto(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			joy_send_info_.setPedalAuto(req->data);
			res->success = joy_send_info_.getPedalAuto();
			res->message = "";
		}

		// ステアの手動入力操作用(joyボードの値をそのまま入力)
		//[in] req:joyボードに入力するステア値
		//[out] res:現在はtrueのみを返す
		void serviceSteerInput(const wada_vmc_msgs::srv::ActualInputUnsigned::Request::SharedPtr req, const wada_vmc_msgs::srv::ActualInputUnsigned::Response::SharedPtr res)
		{
			joy_send_info_.setOperationAutoware(false);
			joy_send_info_.clearSteerCommand();
			int16_t st = std::min(std::max(req->input, car_info_.steer_input_right_min_), car_info_.steer_input_left_max_);
			joy_send_info_.addSteerCommand(st, this->now());
			res->success = true;
		}

		// ステアの手動入力操作用(タイヤ角を入力)
		//[in] req:joyボードに入力するタイヤ角
		//[out] res:現在はtrueのみを返す
		void serviceSteerInputTire(const wada_vmc_msgs::srv::ActualInputFloat::Request::SharedPtr req, const wada_vmc_msgs::srv::ActualInputFloat::Response::SharedPtr res)
		{
			const float cmd_tire_angle = req->input;
			int16_t command_steer;
			if(cmd_tire_angle > 0)
			{
				command_steer = cmd_tire_angle * tire_rad_to_steering_can_value_left_slope_ + tire_rad_to_steering_can_value_left_intercept_ + car_info_.steer_input_center_;
				//RCLCPP_INFO(this->get_logger(), "left %d,%f,%lf,%lf",(int)command_steer, cmd_tire_angle, tire_rad_to_steering_can_value_left_slope_, tire_rad_to_steering_can_value_left_intercept_);
			}
			else
			{
				command_steer = cmd_tire_angle * tire_rad_to_steering_can_value_right_slope_ + tire_rad_to_steering_can_value_right_intercept_ + car_info_.steer_input_center_;
				//RCLCPP_INFO(this->get_logger(), "right %d,%f,%lf,%lf",(int)command_steer, cmd_tire_angle, tire_rad_to_steering_can_value_right_slope_, tire_rad_to_steering_can_value_right_intercept_);
			}
			command_steer = std::min(std::max(command_steer, car_info_.steer_input_right_min_), car_info_.steer_input_left_max_);
			//RCLCPP_INFO(this->get_logger(), "steer_cmd %d,%f,%lf,%lf",command_steer, cmd_tire_angle, tire_rad_to_steering_can_value_right_slope_, tire_rad_to_steering_can_value_right_intercept_);

			joy_send_info_.setOperationAutoware(false);
			joy_send_info_.clearSteerCommand();
			joy_send_info_.addSteerCommand(command_steer, this->now());
			res->success = true;
		}

		// ペダルの手動入力操作用(joyボードの値をそのまま入力)
		//[in] req:joyボードに入力するペダル値
		//[out] res:現在はtrueのみを返す
		void servicePedalInput(const wada_vmc_msgs::srv::ActualInputUnsigned::Request::SharedPtr req, const wada_vmc_msgs::srv::ActualInputUnsigned::Response::SharedPtr res)
		{
			joy_send_info_.setOperationAutoware(false);
			joy_send_info_.clearPedalCommand();
			int16_t pd = std::min(std::max(req->input, car_info_.pedal_input_accel_min_), car_info_.pedal_input_brake_max_);
			joy_send_info_.addPedalCommand(pd, this->now());
			res->success = true;
		}

		// ステアDO指令を入力(クラッチ切断指令は別のサービスで行う)
		//[in] req:DO指令
		//[out] res:現在はtrueのみを返す
		void serviceSteerDO(const wada_vmc_msgs::srv::DOInput_Request::SharedPtr req, const wada_vmc_msgs::srv::DOInput_Response::SharedPtr res)
		{
			joy_send_info_.setSteerDO(req->input);
			res->success = joy_send_info_.getSteerDO();
		}

		// ペダルDO指令を入力(クラッチ切断指令は別のサービスで行う)
		//[in] req:DO指令
		//[out] res:現在はtrueのみを返す
		void servicePedalDO(const wada_vmc_msgs::srv::DOInput_Request::SharedPtr req, const wada_vmc_msgs::srv::DOInput_Response::SharedPtr res)
		{
			joy_send_info_.setPedalDO(req->input);
			res->success = joy_send_info_.getPedalDO();
		}

		// ステア自動可否を入力
		//[in] req ステア自動可否指令
		//[out] res:入力した値を返す
		void serviceSteerAutoPermission(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			joy_send_info_.setSteerAutoPermission(req->data);
			res->success = joy_send_info_.getSteerAutoPermission();
			res->message = "";
		}

		// ペダル自動可否を入力
		//[in] req ペダル自動可否指令
		//[out] res:入力した値を返す
		void servicePedalAutoPermission(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			joy_send_info_.setPedalAutoPermission(req->data);
			res->success = joy_send_info_.getPedalAutoPermission();
			res->message = "";
		}

		// ステアフェイルセーフを入力
		//[in] req ステアフェイルセーフ指令
		//[out] res:入力した値を返す
		void serviceSteerFailsafe(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			joy_send_info_.setSteerFailsafe(req->data);
			res->success = joy_send_info_.getSteerFailsafe();
			res->message = "";
		}

		// ペダルフェイルセーフを入力
		//[in] req ペダルフェイルセーフ指令
		//[out] res:入力した値を返す
		void servicePedalFailsafe(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			joy_send_info_.setPedalFailsafe(req->data);
			res->success = joy_send_info_.getPedalFailsafe();
			res->message = "";
		}

		// ブレーキテイクオーバーを入力
		//[in] req ブレーキていくオーバー指令
		//[out] res:入力した値を返す
		void serviceBrakeTakeover(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			joy_send_info_.setBrakeTakeover(req->data);
			res->success = joy_send_info_.getBrakeTakeover();
			res->message = "";
		}

		// ステアクラッチ切断指令(ステアDO指令1bit)
		//[in] req ステアクラッチ切断指令
		//[out] res:入力した値を返す
		void serviceSteerClutchBan(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			joy_send_info_.setSteerClutchBan(req->data);
			res->success = joy_send_info_.getSteerClutchBan();
			res->message = "";
		}

		// ペダルクラッチ切断指令(ペダルDO指令1bit)
		//[in] req ペダルクラッチ切断指令
		//[out] res:入力した値を返す
		void servicePedalClutchBan(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			joy_send_info_.setPedalClutchBan(req->data);
			res->success = joy_send_info_.getPedalClutchBan();
			res->message = "";
		}

		// 車両操作を手動入力操作かたAUTOWARE操作にする
		//[in] req:運用なし
		//[out] res:運用なし
		void serviceAutowareInputMode(const std_srvs::srv::Empty::Request::SharedPtr, const std_srvs::srv::Empty::Response::SharedPtr)
		{
			joy_send_info_.setOperationAutoware(true);
		}

	private: // ros2 timer
		// 初期ロック解除に回る通常処理タイマー
		void callbackTimer()
		{
			publishID100();
		}

	private: // その他の関数
		// message ID100のデータをros2トピックとして送信
		void publishID100()
		{
			rclcpp::Time ros_nowtime = this->now();
			if (joy_send_info_.getOperationAutoware() == true) // AUTOWAREでの操作になっている場合(サービスからの入力ではない場合)は古い情報を削除
			{
				joy_send_info_.eraseSteerCommand(ros_nowtime);
				joy_send_info_.erasePedalCommand(ros_nowtime);
			}

			{
				// joyボードに送信するmessage ID100番のpublishクラスを準備
				wada_vmc_msgs::msg::Can100_20221111::UniquePtr can100 = std::make_unique<wada_vmc_msgs::msg::Can100_20221111>();
				can100->stamp = ros_nowtime;

				// ステアとペダルの自動設定
				can100->steer_control_mode = (joy_send_info_.getSteerAuto() == true) ? wada_vmc_msgs::msg::Can100_20221111::STEER_AUTO : wada_vmc_msgs::msg::Can100_20221111::STEER_MANUAL;
				can100->pedal_control_mode = (joy_send_info_.getPedalAuto() == true) ? wada_vmc_msgs::msg::Can100_20221111::PEDAL_AUTO : wada_vmc_msgs::msg::Can100_20221111::PEDAL_MANUAL;

				// ペダルコマンド
				int16_t pedal_cmd = joy_send_info_.getPedalCommand(0);
				can100->pedal_cmd = (pedal_cmd == JoyBoardSendInfo::PEDAL_COMMAND_INVALID) ? car_info_.pedal_input_center_ : pedal_cmd;
				can100->pedal_cmd = std::min(std::max(can100->pedal_cmd, car_info_.pedal_input_accel_min_), car_info_.pedal_input_brake_max_);

				// ステアコマンド
				int16_t handle_cmd = joy_send_info_.getSteerCommand(0);
				can100->handle_cmd = (handle_cmd == JoyBoardSendInfo::STEER_COMMAND_INVALID) ? car_info_.steer_input_center_ : handle_cmd;
				can100->handle_cmd = std::min(std::max(can100->handle_cmd, car_info_.steer_input_right_min_), car_info_.steer_input_left_max_);

				// 自動可否
				can100->handle_auto_permission = joy_send_info_.getSteerAutoPermission();
				std_msgs::msg::Bool handle_auto_permission_;
				handle_auto_permission_.data = can100->handle_auto_permission;
				pub_handle_auto_permission_->publish(handle_auto_permission_);
				can100->pedal_auto_permission = joy_send_info_.getPedalAutoPermission();
				std_msgs::msg::Bool pedal_auto_permission_;
				pedal_auto_permission_.data = can100->pedal_auto_permission;
				pub_pedal_auto_permission_->publish(pedal_auto_permission_);
				//can100->pedal_auto_permission = true;
				//can100->handle_auto_permission = true;

				// フェイルセーフ
				can100->handle_failsafe = (can100->handle_auto_permission == false) ? true : false; //joy_send_info_.getSteerFailsafe();
				can100->pedal_failsafe = (can100->pedal_auto_permission == false) ? true : false; //joy_send_info_.getPedalFailsafe();
				//can100->handle_failsafe = false;
				//can100->pedal_failsafe = false;

				// ブレーキテイクオーバー
				can100->breke_takeover_latch = joy_send_info_.getBrakeTakeover();

				// DO指令
				can100->handle_do = joy_send_info_.getSteerDO();
				can100->pedal_do = joy_send_info_.getPedalDO();

				// クラッチ切断指令
				can100->handle_clutch_ban = joy_send_info_.getSteerClutchBan();
				can100->pedal_clutch_ban = joy_send_info_.getPedalClutchBan();

				pub_can100_->publish(std::move(can100));

				// コントロールモード
				autoware_auto_vehicle_msgs::msg::ControlModeReport control_report;
				control_report.stamp = ros_nowtime;
				bool pedal_control = (joy_receive_info_.get501(0).clutch == true && joy_receive_info_.get501(0).drive_mode == wada_vmc_msgs::msg::Can501_20221111::DRIVE_MODE_AUTO);
				bool handle_control = (joy_receive_info_.get502(0).clutch == true && joy_receive_info_.get502(0).handle_mode == wada_vmc_msgs::msg::Can502_20221111::HANDLE_MODE_AUTO);
				if (pedal_control == true || handle_control == true)
					control_report.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
				else
					control_report.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
				pub_control_mode_->publish(control_report);
			}
		}

	public:
		VmcCalculator20221111(const rclcpp::NodeOptions &node_option)
			: rclcpp::Node("wada_vmc_calculation", node_option), joy_receive_info_(this->now()), joy_send_info_(this->now()), life_cycle_(1, 0), prev_steer_cmd_(1024)
		{
			this->declare_parameter<int>("steer_input_center", 1024);
			this->declare_parameter<int>("steer_input_left_max", 1941);
			this->declare_parameter<int>("steer_input_right_min", 0);
			this->declare_parameter<int>("pedal_input_center", 1024);
			this->declare_parameter<int>("pedal_input_brake_max", 1500);
			this->declare_parameter<int>("pedal_input_accel_min", 100);
			this->declare_parameter<double>("cmd_brake_scale_factor", 1.0);

			car_info_.steer_input_center_ = static_cast<int16_t>(this->get_parameter("steer_input_center").as_int());
			car_info_.steer_input_left_max_ = static_cast<int16_t>(this->get_parameter("steer_input_left_max").as_int());
			car_info_.steer_input_right_min_ = static_cast<int16_t>(this->get_parameter("steer_input_right_min").as_int());
			car_info_.pedal_input_center_ = static_cast<int16_t>(this->get_parameter("pedal_input_center").as_int());
			car_info_.pedal_input_accel_min_ = static_cast<int16_t>(this->get_parameter("pedal_input_accel_min").as_int());
			car_info_.pedal_input_brake_max_ = static_cast<int16_t>(this->get_parameter("pedal_input_brake_max").as_int());
			car_info_.cmd_brake_scale_factor_ = get_parameter("cmd_brake_scale_factor").as_double();

			/*RCLCPP_INFO(this->get_logger(), "steer_input_center,%d", car_info_.steer_input_center_);
			RCLCPP_INFO(this->get_logger(), "steer_input_left_max,%d", car_info_.steer_input_left_max_);
			RCLCPP_INFO(this->get_logger(), "steer_input_right_min,%d", car_info_.steer_input_right_min_);
			RCLCPP_INFO(this->get_logger(), "pedal_input_center,%d", car_info_.pedal_input_center_);
			RCLCPP_INFO(this->get_logger(), "pedal_input_accel_min,%d", car_info_.pedal_input_accel_min_);
			RCLCPP_INFO(this->get_logger(), "pedal_input_brake_max,%d", car_info_.pedal_input_brake_max_);*/
			RCLCPP_INFO(this->get_logger(), "cmd_brake_scale_factor,%lf", car_info_.cmd_brake_scale_factor_);

			this->declare_parameter<double>("tire_rad_to_steering_can_value_left_slope", 0.0);
			this->declare_parameter<double>("tire_rad_to_steering_can_value_left_intercept", 0.0);
			this->declare_parameter<double>("tire_rad_to_steering_can_value_right_slope", 0.0);
			this->declare_parameter<double>("tire_rad_to_steering_can_value_right_intercept", 0.0);

			tire_rad_to_steering_can_value_left_slope_ = get_parameter("tire_rad_to_steering_can_value_left_slope").as_double();
			tire_rad_to_steering_can_value_left_intercept_ = get_parameter("tire_rad_to_steering_can_value_left_intercept").as_double();
			tire_rad_to_steering_can_value_right_slope_ = get_parameter("tire_rad_to_steering_can_value_right_slope").as_double();
			tire_rad_to_steering_can_value_right_intercept_ = get_parameter("tire_rad_to_steering_can_value_right_intercept").as_double();

			// joyボード送信用publisher
			pub_can100_ = this->create_publisher<wada_vmc_msgs::msg::Can100_20221111>("can100", rclcpp::QoS(1));
			pub_handle_auto_permission_ = this->create_publisher<std_msgs::msg::Bool>("handle_auto_permission", 1);
			pub_pedal_auto_permission_ = this->create_publisher<std_msgs::msg::Bool>("pedal_auto_permission", 1);

			// autoware用publisher
			pub_control_mode_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS(1));
			pub_tmp_int16_ = this->create_publisher<std_msgs::msg::Int16>("tmp_int16", rclcpp::QoS(1));

			// joyボード受信用subscriber
			sub_can501_ = this->create_subscription<wada_vmc_msgs::msg::Can501_20221111>("can501", rclcpp::SensorDataQoS(),
				std::bind(&VmcCalculator20221111::callbackCan501, this, std::placeholders::_1));
			sub_can502_ = this->create_subscription<wada_vmc_msgs::msg::Can502_20221111>("can502", rclcpp::SensorDataQoS(),
				std::bind(&VmcCalculator20221111::callbackCan502, this, std::placeholders::_1));

			// autowareからのコールバック
			sub_steer_cmd_ = this->create_subscription<wada_vmc_msgs::msg::SteerCmd>("steer_cmd", rclcpp::QoS(1),
				std::bind(&VmcCalculator20221111::callbackSteerCmd, this, std::placeholders::_1));
			sub_actuation_cmd_ = this->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>("/control/command/actuation_cmd", rclcpp::QoS(1),
				std::bind(&VmcCalculator20221111::callbackActuationCmd, this, std::placeholders::_1));
			//sub_autoware_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>("/system/operation_mode/state", rclcpp::QoS(1).transient_local(),
			//	std::bind(&VmcCalculator20221111::callbackAutowareState, this, std::placeholders::_1));
			sub_pedal_clutch_latch_ = this->create_subscription<std_msgs::msg::Bool>("pedal_clutch_latch", rclcpp::QoS(1),
				std::bind(&VmcCalculator20221111::callbackPedalClutchLatch, this, std::placeholders::_1));
			sub_auto_permission_ = this->create_subscription<auto_permission_msgs::msg::AutoPermission>("auto_permission", rclcpp::QoS(1),
				std::bind(&VmcCalculator20221111::callbackAutoPermission, this, std::placeholders::_1));

			// joyボード制御用サービス
			srv_steer_auto_ = this->create_service<std_srvs::srv::SetBool>("steer_auto_switch",
				std::bind(&VmcCalculator20221111::serviceSteerAuto, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_pedal_auto_ = this->create_service<std_srvs::srv::SetBool>("pedal_auto_switch",
				std::bind(&VmcCalculator20221111::servicePedalAuto, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_steer_input_ = this->create_service<wada_vmc_msgs::srv::ActualInputUnsigned>("steer_input",
				std::bind(&VmcCalculator20221111::serviceSteerInput, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_steer_input_tier_ = this->create_service<wada_vmc_msgs::srv::ActualInputFloat>("steer_input_tire",
				std::bind(&VmcCalculator20221111::serviceSteerInputTire, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_pedal_input_ = this->create_service<wada_vmc_msgs::srv::ActualInputUnsigned>("pedal_input",
				std::bind(&VmcCalculator20221111::servicePedalInput, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_steer_do_ = this->create_service<wada_vmc_msgs::srv::DOInput>("steer_do",
				std::bind(&VmcCalculator20221111::serviceSteerDO, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_pedal_do_ = this->create_service<wada_vmc_msgs::srv::DOInput>("pedal_do",
				std::bind(&VmcCalculator20221111::servicePedalDO, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_steer_auto_permission_ = this->create_service<std_srvs::srv::SetBool>("steer_auto_permission",
				std::bind(&VmcCalculator20221111::serviceSteerAutoPermission, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_pedal_auto_permission_ = this->create_service<std_srvs::srv::SetBool>("pedal_auto_permission",
				std::bind(&VmcCalculator20221111::servicePedalAutoPermission, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_steer_failsafe_ = this->create_service<std_srvs::srv::SetBool>("steer_failsafe",
				std::bind(&VmcCalculator20221111::serviceSteerFailsafe, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_pedal_failsafe_ = this->create_service<std_srvs::srv::SetBool>("pedal_failsafe",
				std::bind(&VmcCalculator20221111::servicePedalFailsafe, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_brake_takeover_ = this->create_service<std_srvs::srv::SetBool>("brake_takeover",
				std::bind(&VmcCalculator20221111::serviceBrakeTakeover, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_steer_clutch_ban_ = this->create_service<std_srvs::srv::SetBool>("steer_clutc_ban",
				std::bind(&VmcCalculator20221111::serviceSteerClutchBan, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_pedal_clutch_ban_ = this->create_service<std_srvs::srv::SetBool>("pedal_clutc_ban",
				std::bind(&VmcCalculator20221111::servicePedalClutchBan, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());
			srv_autoware_input_mode_ = this->create_service<std_srvs::srv::Empty>("autoware_input_mode",
				std::bind(&VmcCalculator20221111::serviceAutowareInputMode, this, std::placeholders::_1, std::placeholders::_2),
				rclcpp::ServicesQoS().get_rmw_qos_profile());

			// ros2 timer
			timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(TIMER_MESSAGE100_HZ).period(),
				std::bind(&VmcCalculator20221111::callbackTimer, this));

			joy_send_info_.setSteerAuto(true);
			joy_send_info_.setPedalAuto(true);
		}
	};
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<wada_vmc::VmcCalculator20221111> node = std::make_shared<wada_vmc::VmcCalculator20221111>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
