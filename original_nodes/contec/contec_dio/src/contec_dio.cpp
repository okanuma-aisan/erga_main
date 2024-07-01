#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

//autoware header
#include <wada_vmc_msgs/msg/can501_20221111.hpp>
#include <wada_vmc_msgs/msg/can502_20221111.hpp>
#include <auto_permission_msgs/msg/auto_permission.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
using autoware_adapi_v1_msgs::msg::OperationModeState;

#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>//AUTOWAREからの指令ウィンカー指示
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>//CONTECデバイスからの実際ウィンカー状態

//contec header
#include "cdio.h"
#include "autoware_receive.hpp"//AUTOWAREからの指令ウィンカー情報を保持するクラス
#include "contec_dio_status.hpp"//CONTECデバイスからの実際ウィンカー情報を保持するクラス

//#include <autoware_can_msgs/Alighting.h>

namespace contec
{

class ContecDio : public rclcpp::Node
{
private://各種定数
	static constexpr unsigned int RECEIVE_TIME_HZ = 30;//CONTEC DIOから受信するタイマーの周期
	const rclcpp::Duration LIFE_CYCLE = rclcpp::Duration(0, (uint32_t)(1.0 * 1E9));
	auto_permission_msgs::msg::AutoPermission::SharedPtr last_permission_;

private://ros2 subscriber
	rclcpp::Subscription<wada_vmc_msgs::msg::Can501_20221111>::SharedPtr sub_can501_;//CAN送信番号0x501
	rclcpp::Subscription<wada_vmc_msgs::msg::Can502_20221111>::SharedPtr sub_can502_;//CAN送信番号0x501
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_handle_clutch_ban_;//ハンドルクラッチ断の指令
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_handle_set_;//ハンドルセット指令
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_handle_led_;//ハンドルLED指令
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_handle_failsafe_;//ハンドルフェイルセーフ指令
	rclcpp::Subscription<auto_permission_msgs::msg::AutoPermission>::SharedPtr sub_auto_permission_;//自動運転許可フラグ
	//rclcpp::Subscription<OperationModeState>::SharedPtr sub_autoware_state_;//自動運転許可フラグ
	rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr sub_blinker_;//ウィンカーコマンド
	
private://ros publisher
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_in_port0bit_;//CONTECデバイスのIN０番ポートビット列表示
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_in_port1bit_;//CONTECデバイスのIN１番ポートビット列表示
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_out_port0bit_;//CONTECデバイスのOUT０番ポートビット列表示
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_out_port1bit_;//CONTECデバイスのOUT１番ポートビット列表示
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_echo_port0bit_;//CONTECデバイスのecho０番ポートビット列表示
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_echo_port1bit_;//CONTECデバイスのecho１番ポートビット列表示
	rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr pub_blinker_;//CONTECデバイスからのウィンカー情報をpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_alighting_;//CONTECデバイスからの降車ボタン情報をpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_handle_main_;//CONTECデバイスからのハンドルメイン情報をpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_pedal_main_;//CONTECデバイスからのペダルメイン情報をpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_handle_takeover_;//CONTECデバイスからのハンドルテイクオーバー情報をpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_accel_takeover_;//CONTECデバイスからのアクセルテイクオーバー情報をpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_brake_takeover_;//CONTECデバイスからのブレーキテイクオーバー情報をpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_handle_set_;//CONTECデバイスからのハンドルセット情報をpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_handle_disconnection_;//CONTECデバイスからのハンドル断線情報をpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_pedal_disconnection_;//CONTECデバイスからのペダル断線情報をpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_pedal_clutch_latch_;//CONTECデバイスからのペダルクラッチ保持情報をpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_blinker_cancel_;//CONTECデバイスからのウィンカーキャンセル情報をpublish
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_safety_info_;//クラッチ断判定を行うbool値群をビット情報にしてpublish
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_log_trigger_;//log出力を行うトリガー情報をpublish

private://ros2 timer
	rclcpp::TimerBase::SharedPtr receive_timer_;//CONTECからのデータを受信するタイマー

private://Contecの接続用メンバ変数
	short device_id_;//connectするCONTECデバイスID
	bool is_connect_;//CONTECデバイスにconnect出来たらtrue

private://AUTOWARE用メンバ変数
	AutowareReceive autoware_receive_;//AUTOWAREから受信したデータを保持する

private://CONTEC DIOからの受信情報
	ContecDioStatus contec_dio_status_;//CONTECデバイスから受信したデータを保持する
	bool blinker_intervention_;//ウィンカーの介入判定　介入があるとtrue
	uint8_t prev_blinker_command_;//前回のAUTOWAREからのウィンカー指令
	double can_update_time_th_sec_;//CAN通信の更新時間しきい値
	uint8_t prev_out_port0data_ = 0;//前回CONTECのOUTに送信したport0データ
	uint8_t prev_out_port1data_ = 0;//前回CONTECのOUTに送信したport1データ
	bool out_first1_ = false;
	bool out_first2_ = false;
	auto_permission_msgs::msg::AutoPermission auto_perm_;//自動運転許可フラグ

private://ros callback
	//joy boardペダル情報
	void callbackCan501(const wada_vmc_msgs::msg::Can501_20221111::SharedPtr can501)
	{
		autoware_receive_.setCan501(*can501);
	}

	//joy boardステア情報
	void callbackCan502(const wada_vmc_msgs::msg::Can502_20221111::SharedPtr can502)
	{
		autoware_receive_.setCan502(*can502);
	}

	//ハンドルクラッチ断指令のコールバック
	//[in] msg:空
	void callbackHandleClutchBan(const std_msgs::msg::Empty::SharedPtr)//未使用引数で警告がでるので、変数名を省略することで回避
	{
		autoware_receive_.setHandleClutchBan(true);
	}

	//ハンドルセット指令のコールバック
	//[in] msg:空
	void callbackHandleSet(const std_msgs::msg::Empty::SharedPtr)//未使用引数で警告がでるので、変数名を省略することで回避
	{
		autoware_receive_.setHandleSet(true);
	}

	//ハンドル自動可否LEDのコールバック
	//[in] msg:自動可否判定
	void callbackHandleLED(const std_msgs::msg::Bool::SharedPtr msg)
	{
		autoware_receive_.setHandleLED(msg->data);
	}

	//ハンドルセットのコールバック
	//[in] msg:フェイルセーフ判定
	void callbackHandleFailsafe(const std_msgs::msg::Bool::SharedPtr msg)
	{
		autoware_receive_.setHandleFailsafe(msg->data);
	}

	//自動運転可否情報のコールバック
	void callbackAutoPermission(const auto_permission_msgs::msg::AutoPermission::SharedPtr perm)
	{
		last_permission_ = perm;
		
		if(perm->pedal_permission != auto_permission_msgs::msg::AutoPermission::PERM_OK)
		{
			autoware_receive_.setPedalAutoPermission(false);
			autoware_receive_.setPedalFailsafe(true);
		}
		else autoware_receive_.setPedalAutoPermission(true);

		if(perm->steer_permission != auto_permission_msgs::msg::AutoPermission::PERM_OK)
		{
			autoware_receive_.setHandleAutoPermission(false);
			autoware_receive_.setHandleFailsafe(true);
		}
		else autoware_receive_.setHandleAutoPermission(true);
	}

	/*void callbackAutowareState(const OperationModeState autoware_state)
	{
		//last_permission_->permission != auto_permission_msgs::msg::AutoPermission::PERM_OK ||
		if (autoware_state.mode != OperationModeState::AUTONOMOUS)
		{
			autoware_receive_.setHandleAutoPermission(false);
			autoware_receive_.setPedalAutoPermission(false);
			autoware_receive_.setHandleFailsafe(true);
			autoware_receive_.setPedalFailsafe(true);
		}
		else
		{
			autoware_receive_.setHandleAutoPermission(true);
			autoware_receive_.setPedalAutoPermission(true);
		}
	}*/

	void callbackBlinker(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr blinker)
	{
		autoware_receive_.addBlinker(blinker->command, this->now());
	}

private://ros2 timer
	bool prev_handle_clutch_ban_ = true;//前回のクラッチ断情報
	bool prev_pedal_clutch_ban_ = true;//前回のペダル断情報
	bool prev_accel_takeover_ = false;//前回のアクセルテイクオーバー情報
	bool prev_brake_takeover_ = false;//前回のブレーキテイクオーバー情報
	bool prev_pedal_clutch_latch_ = false;//前回のブレーキテイクオーバー保持情報
	bool handle_disconnection_pushed_ = false;//ハンドル断線情報が過去にONならtrue
	bool pedal_disconnection_pushed_ = false;//ペダル断線情報が過去にONならtrue
	bool handle_main_pushed_ = false;//ハンドルメインが過去にONならtrue
	bool pedal_main_pushed_ = false;//ペダルメインが過去にONならtrue
	bool handle_can_update_time_pushed_ = false;//ハンドル用CAN通信が一定時間来なかったらtrue
	bool pedal_can_update_time_pushed_ = false;//ペダル用CAN通信が一定時間来なかったらtrue
	bool handle_mode_pushed_ = false;//ハンドルモードが過去にONならtrue
	bool pedal_mode_pushed_ = false;//ペダルモードが過去にONならtrue
	bool handle_permission_pushed_ = false;//ハンドル用自動走行許可がなかったらtrue
	bool pedal_permission_pushed_ = false;//ペダル用自動走行許可がなかったらtrue
	bool handle_takeover_pushed_ = false;//ハンドルテイクオーバーが過去に押されたらtrue
	bool accel_takeover_pushed_ = false;//アクセルテイクオーバーが過去に押されたらtrue
	bool brake_takeover_pushed_ = false;//ブレーキテイクオーバーが過去に押されたらtrue
	bool pedal_clutch_latch_change_ = false;//ブレーキテイクオーバー保持が切り替わったらtrue

	//rosタイマーコールバック
	//[in] &e:タイマーイベントクラス
	void callbackReceiveTimer()
	{
		rclcpp::Time nowtime = this->now();

		//CONTECデバイスから0番ポートデータを取得
		uint8_t in_port0data, in_port1data;
		long ret = DioInpByte(device_id_, 0, &in_port0data);
		if(ret != DIO_ERR_SUCCESS)
		{
			RCLCPP_ERROR(this->get_logger(), "error : DioInBackByte : device,%d  port,0  data,%u",
				device_id_, in_port0data);
			return;
		}
		ret = DioInpByte(device_id_, 1, &in_port1data);
		if(ret != DIO_ERR_SUCCESS)
		{
			RCLCPP_ERROR(this->get_logger(), "error : DioInBackByte : device,%d  port,1  data,%u",
				device_id_, in_port0data);
			return;
		}

		uint8_t echo_port0data, echo_port1data;
		ret = DioEchoBackByte(device_id_, 0, &echo_port0data);
		if(ret != DIO_ERR_SUCCESS)
		{
			RCLCPP_ERROR(this->get_logger(), "error : DioEchoBackByte : device,%d  port,0  data,%u",
				device_id_, echo_port0data);
			return;
		}
		ret = DioEchoBackByte(device_id_, 1, &echo_port1data);
		if(ret != DIO_ERR_SUCCESS)
		{
			RCLCPP_ERROR(this->get_logger(), "error : DioEchoBackByte : device,%d  port,1  data,%u",
				device_id_, echo_port1data);
			return;
		}

		//ROS_INFO("buf,%u", in_port0data);

		//------デバイス制御処理-------
		uint8_t command0 = 0, command1 = 0;

		bool in_blinker_cancel = processingBlinkerCancel(in_port0data);//ウィンカーキャンセル
		processingBlinker(in_port1data, in_blinker_cancel, nowtime, command1);//ウィンカー制御
		processingAlighting(in_port0data, nowtime);//降車ボタン状態
		bool in_handle_takeover = processingHandleTakeOver(in_port1data);//ハンドルテイクオーバー
		bool in_accel_takeover = processingAccelTakeOver(in_port1data);//アクセルテイクオーバー
		bool in_brake_takeover = processingBrakeTakeOver(in_port1data);//ブレーキテイクオーバー
		bool in_pedal_clutch_latch = processingPedalClutchLatch(in_port0data);//ペダルクラッチ保持
		bool in_handle_set = processingHandleSet(in_port1data);//ハンドルセット
		bool in_pedal_set = processingPedalSet(in_port0data);//ペダルセット
		bool in_handle_disconnection = processingHandleDisconnection(in_port0data);//ハンドル断線
		bool in_pedal_disconnection = processingPedalDisconnection(in_port0data);//ペダル断線
		bool in_handle_main = processingHandleMain(in_port1data, nowtime);//ハンドルメイン
		bool in_pedal_main = processingPedalMain(in_port1data, nowtime);//ペダルメイン

		bool handle_clutch_ban = prev_handle_clutch_ban_;
		//ハンドル関連
		{
			//ハンドル断線判定
			if(in_handle_disconnection == true)
				handle_disconnection_pushed_ = true;
			if(handle_disconnection_pushed_ == true)
			{
				////std::cout << "handle_disconnection" << std::endl;
				handle_clutch_ban = true;
			}

			//ハンドルメインチェック
			if(in_handle_main == false)
				handle_main_pushed_ = true;
			if(handle_main_pushed_ == true)
			{
				////std::cout << "handle_main false" << std::endl;
				handle_clutch_ban = true;
			}

			//ハンドルCAN更新判定
			rclcpp::Duration ros_timediff = nowtime - autoware_receive_.getCan502().stamp;
			double timediff = ros_timediff.seconds();//ros_timediff.sec + ros_timediff.nsec * 1E-9;
			if(timediff > can_update_time_th_sec_)
				handle_can_update_time_pushed_ = true;
			if(handle_can_update_time_pushed_ == true)
			{
				////std::cout << "handle_can_update" << std::endl;
				handle_clutch_ban = true;
			}

			//ハンドル自動モード判定
			if(autoware_receive_.getCan502().handle_mode != wada_vmc_msgs::msg::Can502_20221111::HANDLE_MODE_AUTO)
				handle_mode_pushed_ = true;
			if(handle_mode_pushed_ == true)
			{
				////std::cout << "handle_auto_mode" << std::endl;
				handle_clutch_ban = true;
			}

			//ハンドル自動可否判定
			if(autoware_receive_.getHandleAutoPermission() == false)
			{
				handle_permission_pushed_ = true;
				//command1 |= ContecDioStatus::DO1_HANDLE_LED;
				//command1 |= ContecDioStatus::DO1_HANDLE_CLUTCH_BAN;
			}
			if(handle_permission_pushed_ == true)
			{
				////std::cout << "handle_permission" << std::endl;
				handle_clutch_ban = true;
			}

			//ハンドルテイクオーバー判定
			if(in_handle_takeover == true)
				handle_takeover_pushed_ = true;
			if(handle_takeover_pushed_ == true)
			{
				////std::cout << "handle_takeover" << std::endl;
				handle_clutch_ban = true;
			}

			//ハンドル自動復帰判定
			//bool handle_set = false;
			if(in_handle_set == true)
			{
				if(in_handle_main == true
					&& autoware_receive_.getCan502().handle_mode == wada_vmc_msgs::msg::Can502_20221111::HANDLE_MODE_AUTO
					&& autoware_receive_.getHandleAutoPermission() == true
					&& in_handle_takeover == false)
				{
					handle_disconnection_pushed_ = false;
					handle_main_pushed_ = false;
					handle_can_update_time_pushed_ = false;
					handle_mode_pushed_ = false;
					handle_permission_pushed_ = false;
					handle_takeover_pushed_ = false;
					//handle_set = true;
					////std::cout << "handle set" << std::endl;
				}
				if(!handle_disconnection_pushed_ && !handle_main_pushed_ && !handle_can_update_time_pushed_
					&& !handle_mode_pushed_ && !handle_permission_pushed_ && !handle_takeover_pushed_)
				{
					//handle_set = true;
					handle_clutch_ban = false;
					autoware_receive_.setHandleFailsafe(false);
				}
			}

			handle_clutch_ban = false;//強制的にバンしない
			handle_permission_pushed_ = false;//強制的にバンしない

			//ハンドルフェイルセーフ判定
			//if(handle_permission_pushed_ == true || handle_clutch_ban == true)
			if(autoware_receive_.getHandleFailsafe() == true)
				command0 |= ContecDioStatus::DO0_HANDLE_FAILSAFE;

			RCLCPP_INFO(get_logger(), "handle_clutch_ban,%d", (int)handle_clutch_ban);

			if(handle_clutch_ban == true)
				command1 |= ContecDioStatus::DO1_HANDLE_CLUTCH_BAN;
			/*if(handle_set == true && handle_clutch_ban == false)
				command1 |= ContecDioStatus::HANDLE_SET_OUTPUT;*/

			if(autoware_receive_.getCan502().stamp == rclcpp::Time(0))//ノード起動時はタイムスタンプ0のダミーデータが入っているので、このときはhandle_can_update_time_pushedの保持をしない　
			{
				handle_can_update_time_pushed_ = false;
			}
		}

		bool pedal_clutch_ban = prev_pedal_clutch_ban_;
		//ペダル関連
		{
			//ペダル断線判定
			if(in_pedal_disconnection == true)
				pedal_disconnection_pushed_ = true;
			if(pedal_disconnection_pushed_ == true)
			{
				//std::cout << "pedal_disconnection" << std::endl;
				pedal_clutch_ban = true;
			}

			//ペダルメインチェック
			if(in_pedal_main == false)
				pedal_main_pushed_ = true;
			if(pedal_main_pushed_ == true)
			{
				//std::cout << "pedal_main false" << std::endl;
				pedal_clutch_ban = true;
			}

			//ペダルCAN更新判定
			rclcpp::Duration ros_timediff = nowtime - autoware_receive_.getCan501().stamp;
			double timediff = ros_timediff.seconds();//ros_timediff.sec + ros_timediff.nsec * 1E-9;
			if(timediff > can_update_time_th_sec_)
				pedal_can_update_time_pushed_ = true;
			if(pedal_can_update_time_pushed_ == true)
			{
				//std::cout << "pedal_can_update" << std::endl;
				pedal_clutch_ban = true;
			}

			//ペダル自動モード判定
			if(autoware_receive_.getCan501().drive_mode != wada_vmc_msgs::msg::Can501_20221111::DRIVE_MODE_AUTO)
				pedal_mode_pushed_ = true;
			if(pedal_mode_pushed_ == true)
			{
				////std::cout << "pedal_auto_mode" << std::endl;
				pedal_clutch_ban = true;
			}

			//ペダル自動可否判定
			if(autoware_receive_.getPedalAutoPermission() == false)
			{
				pedal_permission_pushed_ = true;
				//command1 |= ContecDioStatus::DO1_PEDAL_LED;
				//command1 |= ContecDioStatus::DO1_PEDAL_CLUTCH_BAN;
			}
			if(pedal_permission_pushed_ == true)
			{
				//std::cout << "pedal_permission" << std::endl;
				pedal_clutch_ban = true;
			}

			//アクセルテイクオーバー判定
			/*if(in_accel_takeover == true)
				accel_takeover_pushed_ = true;
			if(accel_takeover_pushed_ == true)
			{
				//std::cout << "accel_takeover" << std::endl;
				pedal_clutch_ban = true;
			}*/
			if(in_accel_takeover == true)
				pedal_clutch_ban = true;
			else if(prev_accel_takeover_ == true)
				in_pedal_set = true;
			//else in_pedal_set = true;

			//ブレーキテイクオ−バー
			if(in_pedal_clutch_latch == true)
			{
				if(in_brake_takeover == true)
					brake_takeover_pushed_ = true;
				if(brake_takeover_pushed_ == true)
				{
					////std::cout << "brake_takeover1" << std::endl;
					pedal_clutch_ban = true;
				}
			}
			else
			{
				brake_takeover_pushed_ = false;
				if(in_brake_takeover == true)
				{
					////std::cout << "brake_takeover2" << std::endl;
					pedal_clutch_ban = true;
				}
				else if(prev_brake_takeover_ == true)
					in_pedal_set = true;
			}

			//ブレーキテイクーオーバー保持が切り替わったら、セットボタンが押されるまでクラッチを切る
			if(in_pedal_clutch_latch == false && in_pedal_clutch_latch != prev_pedal_clutch_latch_)
				pedal_clutch_latch_change_ = true;
			if(pedal_clutch_latch_change_ == true)
			{
				////std::cout << "pedal_clutch_latch_change" << std::endl;
				pedal_clutch_ban = true;
			}

			//RCLCPP_INFO(get_logger(), "%d,%d,%d,%d,%d,%d,%d,%d", (int)pedal_disconnection_pushed_, (int)pedal_main_pushed_, (int)pedal_can_update_time_pushed_,
			//	(int)pedal_mode_pushed_, (int)pedal_permission_pushed_, (int)in_accel_takeover, (int)brake_takeover_pushed_, (int)pedal_clutch_latch_change_);

			if(in_pedal_set == true)
			{
				pedal_clutch_latch_change_ = false;

				if(in_pedal_main == true
					&& autoware_receive_.getCan501().drive_mode == wada_vmc_msgs::msg::Can501_20221111::DRIVE_MODE_AUTO
					&& autoware_receive_.getPedalAutoPermission() == true && in_brake_takeover == false)
				{
					pedal_disconnection_pushed_ = false;
					pedal_main_pushed_ = false;
					pedal_can_update_time_pushed_ = false;
					pedal_mode_pushed_ = false;
					pedal_permission_pushed_ = false;
					accel_takeover_pushed_ = false;
					brake_takeover_pushed_ = false;
				}
				if(!pedal_disconnection_pushed_ && !pedal_main_pushed_ && !pedal_can_update_time_pushed_
					&& !pedal_mode_pushed_ && !pedal_permission_pushed_ && !brake_takeover_pushed_)
				{
					pedal_clutch_ban = false;
					autoware_receive_.setPedalFailsafe(false);
				}
			}

			pedal_clutch_ban = false;//強制的バンしない
			pedal_permission_pushed_ = false;//強制的バンしない

			//ペダルフェイルセーフ判定
			//if(pedal_permission_pushed_ == true || pedal_clutch_ban == true)
			if(autoware_receive_.getPedalFailsafe() == true)
				command0 |= ContecDioStatus::DO0_PEDAL_FAILSAFE;

			if(pedal_clutch_ban == true)
				command1 |= ContecDioStatus::DO1_PEDAL_CLUTCH_BAN;
			//if(pedal_set == true && pedal_clutch_ban == false)
			//	command1 |= ContecDioStatus::PEDAL_SET_OUTPUT;

			if(autoware_receive_.getCan501().stamp == rclcpp::Time(0))//ノード起動時はタイムスタンプ0のダミーデータが入っているので、このときはhandle_can_update_time_pushedの保持をしない　
				pedal_can_update_time_pushed_ = false;

			safetyInfoPublish();
		}

		//command1 |= 0x04;
		if(prev_out_port0data_ != command0 || out_first1_ == false)
		{
			DioOutByte(device_id_, 0, command0);
			prev_out_port0data_ = command0;
			publishPortBit(command0, pub_out_port0bit_);
			out_first1_ = true;
		}
		if(prev_out_port1data_ != command1 || out_first2_ == false)
		{
			DioOutByte(device_id_, 1, command1);
			prev_out_port1data_ = command1;
			publishPortBit(command1, pub_out_port1bit_);
			out_first2_ = true;
		}

		//生存時間を超えた情報を削除
		contec_dio_status_.pops(nowtime);//古いCONTECデバイス情報を削除
		autoware_receive_.pops(nowtime);//古いAUTOWARE情報を削除

		publishPortBit(in_port0data, pub_in_port0bit_);
		publishPortBit(in_port1data, pub_in_port1bit_);
		publishPortBit(echo_port0data, pub_echo_port0bit_);
		publishPortBit(echo_port1data, pub_echo_port1bit_);

		prev_handle_clutch_ban_ = handle_clutch_ban;
		prev_pedal_clutch_ban_ = pedal_clutch_ban;
		prev_accel_takeover_ = in_accel_takeover;
		prev_brake_takeover_ = in_brake_takeover;
		prev_pedal_clutch_latch_ = in_pedal_clutch_latch;

		std_msgs::msg::Bool msg_log_trigger_;
		msg_log_trigger_.data = (in_handle_main == true || in_pedal_main == true);
		pub_log_trigger_->publish(msg_log_trigger_);
	}

private://CONTECデバイスとAUTOWAREの情報から車両操作を行う関数群
	void publishPortBit(const uint8_t buf, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub)
	{
		std::stringstream ss;
		for(int i=7; i>=0; i--)
			ss << ((buf >> i) & 0x1);
		std_msgs::msg::String str;
		str.data = ss.str();
		pub->publish(str);
	}

	//ウィンカーコマンドを処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	//[in] contec_blinker_cencel:CONTECからのウィンカーキャンセル情報
	//[in] nowtime:現在の時間
	//[out] output:ウィンカーコマンドを設定するバッファ
	bool prev_blinker_cancel_ = false;//前回ウィンカーキャンセルされたか？
	void processingBlinker(const uint8_t contec_port_data, const bool contec_blinker_cancel,
		const rclcpp::Time &nowtime, uint8_t &output)
	{
		//今回のウィンカー情報を履歴に加える
		uint8_t now_blinker = contec_port_data & ContecDioStatus::DO1_BLINKER_ACQUISITION;//CONTECデバイスのウィンカー情報 ウィンカ情報は、左が1ビット目、右が2ビット目
		contec_dio_status_.addBlinker(now_blinker, nowtime);//CONTECデバイスから現在のウィンカー情報を追加

		//テイクオーバー判定
		if(now_blinker == ContecDioStatus::DO1_BLINKER_INTERVENTION)
		{
			if(blinker_intervention_ == false)
			{
				DioOutByte(device_id_, 0, 0);
				blinker_intervention_ = true;
			}
		}
		else//ウィンカー指令が存在する場合、今回のウィンカー指令をCONTEC DIOに送る
		{
			const int8_t autoware_blinker_command = autoware_receive_.getBlinker(0);//現在のAUTOWAREからのウィンカー指令
			int8_t blinker_command = autoware_blinker_command;
			const int8_t prev_autoware_blinker_command = (autoware_receive_.blinkerQueSize() < 2) ? -1 : autoware_receive_.getBlinker(1);//１つ前のAUTOWAREからのウィンカー指令

			if(contec_blinker_cancel == true)//CONTECのウィンカーキャンセルがONの場合はウィンカー停止
			{
				blinker_command = -1;
				prev_blinker_cancel_ = true;
			}
			else if(prev_blinker_cancel_ == true && autoware_blinker_command == prev_autoware_blinker_command && autoware_blinker_command != -1)//前回でウィンカーがキャンセルされ、同じウィンカー指示だった場合はウィンカー停止
			{
				blinker_command = -1;
				prev_blinker_cancel_ = true;
			}
			else if(blinker_intervention_ == true)//介入判定がある場合はウィンカー停止
			{
				blinker_command = -1;
				prev_blinker_cancel_ = true;
			}
			else if(autoware_receive_.getHandleAutoPermission() == false)//自動許可がない場合はウィンカー停止
			{
				blinker_command = -1;
				prev_blinker_cancel_ = true;
			}
			else
			{
				prev_blinker_cancel_ = false;
			}

			if(blinker_command != -1)// && now_blinker == 0)// && blinker_command != prev_blinker_command_)
			{
				switch(blinker_command)
				{
				case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT://left
					{
						output |= ContecDioStatus::DO1_BLINKER_LEFT;
						break;
					}
				case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT://right
					{
						output |= ContecDioStatus::DO1_BLINKER_RIGHT;
						break;
					}
				}
			}
		
			prev_blinker_command_ = blinker_command;
		}

		//ウィンカー指令が存在しない場合は、介入判定を解除
		if(now_blinker == ContecDioStatus::DO1_BLINKER_DISABLE)
			blinker_intervention_ = false;

		//ウィンカー情報をトピックとしてpublish
		autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport blinker_report;
		blinker_report.stamp = nowtime;
		switch(now_blinker)
		{
		case ContecDioStatus::DI1_BLINKER_LEFT:
			blinker_report.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
			break;
		case ContecDioStatus::DI1_BLINKER_RIGHT:
			blinker_report.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
			break;
		case ContecDioStatus::DI1_BLINKER_DISABLE:
			blinker_report.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
			break;
		default:
			blinker_report.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE;
			break;
		}
		pub_blinker_->publish(blinker_report);
	}

	//降車ボタン情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	//[in] nowtime:現在の時間
	void processingAlighting(const uint8_t contec_port_data, const rclcpp::Time &nowtime)
	{
		uint8_t now_alighting = contec_port_data & ContecDioStatus::DI0_ALIGHTING;//降車ボタン情報を取得
		contec_dio_status_.addAlighting(now_alighting, nowtime);//降車ボタン情報履歴を追加

		//降車ボタン情報をトピックとしてpublish
		std_msgs::msg::Bool aligthting_report;
		aligthting_report.data = ((now_alighting & ContecDioStatus::DI0_ALIGHTING) == 0) ? false : true;
		pub_alighting_->publish(aligthting_report);
	}

	//ハンドルメイン情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	//[in] nowtime:現在の時間
	bool processingHandleMain(const uint8_t contec_port_data, const rclcpp::Time &nowtime)
	{
		uint8_t now_handle_main = contec_port_data & ContecDioStatus::DI1_HANDLE_MAIN;//ハンドルメイン情報を取得
		contec_dio_status_.addHandleMain(now_handle_main, nowtime);//ハンドルメイン情報履歴を追加

		//ハンドルメイン情報をトピックとしてpublish
		std_msgs::msg::Bool handle_main_report;
		handle_main_report.data = ((now_handle_main & ContecDioStatus::DI1_HANDLE_MAIN) == 0) ? false : true;
		pub_handle_main_->publish(handle_main_report);

		return handle_main_report.data;
	}

	//ペダルメイン情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	//[in] nowtime:現在の時間
	bool processingPedalMain(const uint8_t contec_port_data, const rclcpp::Time &nowtime)
	{
		uint8_t now_pedal_main = contec_port_data & ContecDioStatus::DI1_PEDAL_MAIN;//ハンドルメイン情報を取得
		contec_dio_status_.addPedalMain(now_pedal_main, nowtime);//ハンドルメイン情報履歴を追加

		//ハンドルメイン情報をトピックとしてpublish
		std_msgs::msg::Bool pedal_main_report;
		pedal_main_report.data = ((now_pedal_main & ContecDioStatus::DI1_PEDAL_MAIN) == 0) ? false : true;
		pub_pedal_main_->publish(pedal_main_report);

		return pedal_main_report.data;
	}

	//ハンドルテイクオーバー情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	bool processingHandleTakeOver(const uint8_t contec_port_data)
	{
		uint8_t now_handle_takeover = contec_port_data & ContecDioStatus::DI1_HANDLE_TAKEOVER;//ハンドルテイクオーバー情報を取得

		//ハンドルテイクオーバー情報をトピックとしてpublish
		std_msgs::msg::Bool handle_takeover_report;
		handle_takeover_report.data = ((now_handle_takeover & ContecDioStatus::DI1_HANDLE_TAKEOVER) == 0) ? false : true;
		pub_handle_takeover_->publish(handle_takeover_report);

		return handle_takeover_report.data;
	}

	//アクセルテイクオーバー情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	bool processingAccelTakeOver(const uint8_t contec_port_data)
	{
		uint8_t now_accel_takeover = contec_port_data & ContecDioStatus::DI1_ACCEL_TAKEOVER;//アクセルテイクオーバー情報を取得

		//アクセルテイクオーバー情報をトピックとしてpublish
		std_msgs::msg::Bool accel_takeover_report;
		accel_takeover_report.data = ((now_accel_takeover & ContecDioStatus::DI1_ACCEL_TAKEOVER) == 0) ? false : true;
		pub_accel_takeover_->publish(accel_takeover_report);

		return accel_takeover_report.data;
	}

	//ブレーキテイクオーバー情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	bool processingBrakeTakeOver(const uint8_t contec_port_data)
	{
		uint8_t now_brake_takeover = contec_port_data & ContecDioStatus::DI1_BRAKE_TAKEOVER;//ブレーキテイクオーバー情報を取得

		//ブレーキテイクオーバー情報をトピックとしてpublish
		std_msgs::msg::Bool brake_takeover_report;
		brake_takeover_report.data = ((now_brake_takeover & ContecDioStatus::DI1_BRAKE_TAKEOVER) == 0) ? false : true;
		pub_brake_takeover_->publish(brake_takeover_report);

		return brake_takeover_report.data;
	}

	//ペダルクラッチ保持情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	bool processingPedalClutchLatch(const uint8_t contec_port_data)
	{
		uint8_t latch = contec_port_data & ContecDioStatus::DI0_PEDAL_CLUTCH_LATCH;//ペダルクラッチ保持情報を取得

		//ペダルクラッチ保持情報をトピックとしてpublish
		std_msgs::msg::Bool pedal_clutch_latch_report;
		pedal_clutch_latch_report.data = ((latch & ContecDioStatus::DI0_PEDAL_CLUTCH_LATCH) == 0) ? false : true;
		pub_pedal_clutch_latch_->publish(pedal_clutch_latch_report);

		return pedal_clutch_latch_report.data;
	}

	//ハンドルセット情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	bool processingHandleSet(const uint8_t contec_port_data)
	{
		uint8_t now_handle_set = contec_port_data & ContecDioStatus::DI1_HANDLE_SET;//ハンドルセット情報を取得
		//contec_dio_status_.addAlighting(now_handle_takeover, nowtime);//ハンドルセット情報履歴を追加

		//ハンドルセット情報をトピックとしてpublish
		std_msgs::msg::Bool handle_set_report;
		handle_set_report.data = ((now_handle_set & ContecDioStatus::DI1_HANDLE_SET) == 0) ? false : true;
		pub_handle_set_->publish(handle_set_report);

		return handle_set_report.data;
	}

	//ペダルセット情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	bool processingPedalSet(const uint8_t contec_port_data)
	{
		uint8_t now_pedal_set = contec_port_data & ContecDioStatus::DI0_PEDAL_SET;//ハンドルセット情報を取得

		//ハンドルセット情報をトピックとしてpublish
		std_msgs::msg::Bool pedal_set_report;
		pedal_set_report.data = ((now_pedal_set & ContecDioStatus::DI0_PEDAL_SET) == 0) ? false : true;
		pub_handle_set_->publish(pedal_set_report);

		return pedal_set_report.data;
	}

	//ハンドル断線情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	bool processingHandleDisconnection(const uint8_t contec_port_data)
	{
		uint8_t now_handle_disconnection = contec_port_data & ContecDioStatus::DI0_HANDLE_DISCONNECTION;//ハンドル断線情報を取得

		//ハンドル断線情報をトピックとしてpublish
		std_msgs::msg::Bool handle_disconnection_report;
		handle_disconnection_report.data = ((now_handle_disconnection & ContecDioStatus::DI0_HANDLE_DISCONNECTION) == 0) ? false : true;
		pub_handle_disconnection_->publish(handle_disconnection_report);

		return handle_disconnection_report.data;
	}

	//ペダル断線情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	bool processingPedalDisconnection(const uint8_t contec_port_data)
	{
		uint8_t now_pedal_disconnection = contec_port_data & ContecDioStatus::DI0_PEDAL_DISCONNECTION;//ペダル断線情報を取得

		//ペダル断線情報をトピックとしてpublish
		std_msgs::msg::Bool pedal_disconnection_report;
		pedal_disconnection_report.data = ((now_pedal_disconnection & ContecDioStatus::DI0_PEDAL_DISCONNECTION) == 0) ? false : true;
		pub_pedal_disconnection_->publish(pedal_disconnection_report);

		return pedal_disconnection_report.data;
	}

	//ウィンカーキャンセル情報を処理する
	//[in] contec_port_data:contecデバイスポートから取得した1BYTE情報
	bool processingBlinkerCancel(const uint8_t contec_port_data)
	{
		uint8_t now_blinker_cancel = contec_port_data & ContecDioStatus::DI0_BLINKER_CANCEL;//ハンドル断線情報を取得

		//ハンドル断線情報をトピックとしてpublish
		std_msgs::msg::Bool blinker_cancel_report;
		blinker_cancel_report.data = ((now_blinker_cancel & ContecDioStatus::DI0_BLINKER_CANCEL) == 0) ? false : true;
		pub_blinker_cancel_->publish(blinker_cancel_report);

		return blinker_cancel_report.data;
	}

private://その他
	//クラッチ断判定を行うbool値群をビット情報にしてpublish
	void safetyInfoPublish()
	{
		std::stringstream ss;

		if(handle_disconnection_pushed_) ss << '1';
		else ss << '0';
		if(pedal_disconnection_pushed_) ss << '1';
		else ss << '0';
		if(handle_main_pushed_) ss << '1';
		else ss << '0';
		if(pedal_main_pushed_) ss << '1';
		else ss << '0';
		if(handle_can_update_time_pushed_) ss << '1';
		else ss << '0';
		if(pedal_can_update_time_pushed_) ss << '1';
		else ss << '0';
		if(handle_mode_pushed_) ss << '1';
		else ss << '0';
		if(pedal_mode_pushed_) ss << '1';
		else ss << '0';
		if(handle_permission_pushed_) ss << '1';
		else ss << '0';
		if(pedal_permission_pushed_) ss << '1';
		else ss << '0';
		if(handle_takeover_pushed_) ss << '1';
		else ss << '0';
		if(accel_takeover_pushed_) ss << '1';
		else ss << '0';
		if(brake_takeover_pushed_) ss << '1';
		else ss << '0';
		if(pedal_clutch_latch_change_) ss << '1';
		else ss << '0';

		std_msgs::msg::String msg;
		msg.data = ss.str();
		pub_safety_info_->publish(msg);
	}

public://コンストラクタ
	ContecDio(const rclcpp::NodeOptions &node_option)
		: rclcpp::Node("contec_dio", node_option)
		, device_id_(0)
		, is_connect_(false)
		, autoware_receive_(this->now(), LIFE_CYCLE, (int8_t)-1)
		, contec_dio_status_(this->now(), LIFE_CYCLE)
		, blinker_intervention_(false)
		, prev_blinker_command_(-1)
	{
		const std::string device_name = this->declare_parameter<std::string>("device_name", "DIO000");
		can_update_time_th_sec_ = this->declare_parameter<double>("can_update_time_th_sec", 0.3);

		sub_can501_ = this->create_subscription<wada_vmc_msgs::msg::Can501_20221111>("pedal_info", rclcpp::SensorDataQoS(),
			std::bind(&ContecDio::callbackCan501, this, std::placeholders::_1));
		sub_can502_ = this->create_subscription<wada_vmc_msgs::msg::Can502_20221111>("steer_info", rclcpp::SensorDataQoS(),
			std::bind(&ContecDio::callbackCan502, this, std::placeholders::_1));
		sub_auto_permission_ = this->create_subscription<auto_permission_msgs::msg::AutoPermission>("auto_permission", rclcpp::QoS(1),
			std::bind(&ContecDio::callbackAutoPermission, this, std::placeholders::_1));
		//sub_autoware_state_ = this->create_subscription<OperationModeState>("/system/operation_mode/state", rclcpp::QoS(1).transient_local(),
		//	std::bind(&ContecDio::callbackAutowareState, this, std::placeholders::_1));
		sub_blinker_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>("/control/command/turn_indicators_cmd", rclcpp::QoS(1),
			std::bind(&ContecDio::callbackBlinker, this, std::placeholders::_1));

		sub_handle_clutch_ban_ = this->create_subscription<std_msgs::msg::Empty>("set_handle_clutch_ban", rclcpp::QoS(1),
			std::bind(&ContecDio::callbackHandleClutchBan, this, std::placeholders::_1));
		sub_handle_set_ = this->create_subscription<std_msgs::msg::Empty>("set_handle_set", rclcpp::QoS(1),
			std::bind(&ContecDio::callbackHandleSet, this, std::placeholders::_1));
		sub_handle_led_ = this->create_subscription<std_msgs::msg::Bool>("set_handle_led", rclcpp::QoS(1),
			std::bind(&ContecDio::callbackHandleLED, this, std::placeholders::_1));
		sub_handle_failsafe_ = this->create_subscription<std_msgs::msg::Bool>("set_handle_failsafe", rclcpp::QoS(1),
			std::bind(&ContecDio::callbackHandleFailsafe, this, std::placeholders::_1));

		pub_in_port0bit_ = this->create_publisher<std_msgs::msg::String>("in_port0bit", rclcpp::QoS(1));
		pub_in_port1bit_ = this->create_publisher<std_msgs::msg::String>("in_port1bit", rclcpp::QoS(1));
		pub_out_port0bit_ = this->create_publisher<std_msgs::msg::String>("out_port0bit", rclcpp::QoS(1));
		pub_out_port1bit_ = this->create_publisher<std_msgs::msg::String>("out_port1bit", rclcpp::QoS(1));
		pub_echo_port0bit_ = this->create_publisher<std_msgs::msg::String>("echo_port0bit", rclcpp::QoS(1));
		pub_echo_port1bit_ = this->create_publisher<std_msgs::msg::String>("echo_port1bit", rclcpp::QoS(1));
		pub_blinker_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>("/vehicle/status/turn_indicators_status", rclcpp::QoS(1));
		pub_alighting_ = this->create_publisher<std_msgs::msg::Bool>("car_alighting", rclcpp::QoS(1));
		pub_handle_main_ = this->create_publisher<std_msgs::msg::Bool>("handle_main", rclcpp::QoS(1));
		pub_pedal_main_ = this->create_publisher<std_msgs::msg::Bool>("pedal_main", rclcpp::QoS(1));
		pub_handle_takeover_ = this->create_publisher<std_msgs::msg::Bool>("handle_takeover", rclcpp::QoS(1));
		pub_accel_takeover_ = this->create_publisher<std_msgs::msg::Bool>("accel_takeover", rclcpp::QoS(1));
		pub_brake_takeover_ = this->create_publisher<std_msgs::msg::Bool>("brake_takeover", rclcpp::QoS(1));
		pub_handle_set_ = this->create_publisher<std_msgs::msg::Bool>("handle_set", rclcpp::QoS(1));
		pub_handle_disconnection_ = this->create_publisher<std_msgs::msg::Bool>("handle_disconnection", rclcpp::QoS(1));
		pub_pedal_disconnection_ = this->create_publisher<std_msgs::msg::Bool>("pedal_disconnection", rclcpp::QoS(1));
		pub_pedal_clutch_latch_ = this->create_publisher<std_msgs::msg::Bool>("pedal_clutch_latch", rclcpp::QoS(1));
		pub_blinker_cancel_ = this->create_publisher<std_msgs::msg::Bool>("blinker_cancel", rclcpp::QoS(1));
		pub_safety_info_ = this->create_publisher<std_msgs::msg::String>("safety_info", rclcpp::QoS(1));
		pub_log_trigger_ = this->create_publisher<std_msgs::msg::Bool>("log_trigger", rclcpp::QoS(1));

		long ret = DioInit(const_cast<char*>(device_name.c_str()), &device_id_);//初期化
		if(ret != DIO_ERR_SUCCESS)
		{
			RCLCPP_ERROR(this->get_logger(), "error : DioInit, %ld    device name,%s", ret, device_name.c_str());
			return;
		}

		RCLCPP_INFO(this->get_logger(), "Contec device connect OK");
		is_connect_ = true;

		//CONTEC起動時はクラッチは切る
		DioOutByte(device_id_, 1, ContecDioStatus::DO1_HANDLE_CLUTCH_BAN | ContecDioStatus::DO1_PEDAL_CLUTCH_BAN);

		receive_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Rate(RECEIVE_TIME_HZ).period(), std::bind(&ContecDio::callbackReceiveTimer, this));
	}

	~ContecDio()
	{
		DioOutByte(device_id_, 0, 0);
		DioOutByte(device_id_, 1, 0);

		long ret =DioExit(device_id_);
		if(ret != DIO_ERR_SUCCESS)
			RCLCPP_ERROR(this->get_logger(), "error : DioExit");
		else
			RCLCPP_INFO(this->get_logger(), "Contec device disconnect OK");
	}
};

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(contec::ContecDio)
