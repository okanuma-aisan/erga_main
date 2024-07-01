#ifndef WADA_VMC_JOYBOARD_SEND_INFO
#define WADA_VMC_JOYBOARD_SEND_INFO

#include <rclcpp/rclcpp.hpp>
#include <float.h>

namespace wada_vmc
{
	//joyボード送信情報
	class JoyBoardSendInfo
	{
	public://各種定数
		static constexpr int16_t STEER_COMMAND_INVALID = INT16_MAX;//steer commandがこの値の場合はコマンド数値として扱わない
		static constexpr int16_t PEDAL_COMMAND_INVALID = INT16_MAX;//pedal commandがこの値の場合はコマンド数値として扱わない
		static constexpr float CONTROL_COMMAND_INVALID = FLT_MAX;//control commandのspeedがこの値の場合はコマンド数値として扱わない
		
	private://joyボード送信情報
		bool operation_autoware_;//AUTOWRAERで車両操作を行うならtrue、そうでないならfalse
		bool operation_blnker_;//AUTOWAREDEウィンカー操作を行うならtrue、そうでないならfalse
		bool steer_auto_;//ステアを自動運転モードにするならtrue
		bool pedal_auto_;//ペダルを自動運転モードにするならtrue
		uint8_t steer_do_;//ステアDO指令
		uint8_t pedal_do_;//ペダルDO指令
		bool steer_auto_permission_;//ステア自動可否
		bool pedal_auto_permission_;//ステア自動可否
		bool steer_failsafe_;//ステアフェイルセーフ
		bool pedal_failsafe_;//ペダルフェイルセーフ
		bool brake_takeover_;//ブレーキのテイクオーバー保持非保持切り替え（falseで状態保持）
		bool steer_clutch_ban_;//ステアクラッチ切断指令(ステアDO指令1ビット)
		bool pedal_clutch_ban_;//ペダルクラッチ切断指令(ペダルDO指令1ビット)
		std::vector<int16_t> steer_command_;//現在のステア入力指令の履歴
		std::vector<rclcpp::Time> steer_command_time_;//steer_command_を登録した時間の履歴
		std::vector<int16_t> pedal_command_;//現在のペダル入力指令の履歴
		std::vector<rclcpp::Time> pedal_command_time_;//pedal_command_を登録した時間の履歴
		bool pedal_clutch_latch_;//ブレーキペダル保持情報

	private://その他メンバ変数
		rclcpp::Duration life_cycle_;//データ生存時間

	public:
		JoyBoardSendInfo(const rclcpp::Time  &nowtime)
			: operation_autoware_(true)
			, operation_blnker_(true)
			, steer_auto_(false)
			, pedal_auto_(false)
			, steer_do_(0)
			, pedal_do_(0)
			, steer_auto_permission_(false)
			, pedal_auto_permission_(false)
			, steer_failsafe_(false)
			, pedal_failsafe_(false)
			, brake_takeover_(false)
			, steer_clutch_ban_(false)
			, pedal_clutch_ban_(false)
			, pedal_clutch_latch_(false)
			, life_cycle_(1,0)
		{
			steer_command_.push_back(STEER_COMMAND_INVALID);
			steer_command_time_.push_back(nowtime);
			pedal_command_.push_back(PEDAL_COMMAND_INVALID);
			pedal_command_time_.push_back(nowtime);
		}

	public://各種情報をセットする関数
		//operation_autoware_変数の設定と取得
		void setOperationAutoware(const bool flag) {operation_autoware_ = flag;}
		bool getOperationAutoware() const {return operation_autoware_;}

		//operation_blnker_変数の設定と取得
		void setOperationBlnker(const bool flag) {operation_blnker_ = flag;}
		bool getOperationBlinker() const {return operation_blnker_;}

		//steer_auto_変数の設定と取得
		void setSteerAuto(const bool flag) {steer_auto_ = flag;}
		bool getSteerAuto() const {return steer_auto_;}

		//pedal_auto_変数の設定と取得
		void setPedalAuto(const bool flag) {pedal_auto_ = flag;}
		bool getPedalAuto() const {return pedal_auto_;}

		//steer_do_変数の設定と取得
		void setSteerDO(const uint8_t DO) {steer_do_ = DO;}
		uint8_t getSteerDO() {return steer_do_;}

		//pedal_auto_変数の設定と取得
		void setPedalDO(const uint8_t DO) {pedal_do_ = DO;}
		uint8_t getPedalDO() {return pedal_do_;}

		//steer_auto_permission_変数の設定と取得
		void setSteerAutoPermission(const bool flag) {steer_auto_permission_ = flag;}
		bool getSteerAutoPermission() const {return steer_auto_permission_;}

		//pedal_auto_permission_変数の設定と取得
		void setPedalAutoPermission(const bool flag) {pedal_auto_permission_ = flag;}
		bool getPedalAutoPermission() const {return pedal_auto_permission_;}

		//steer_failsafe_変数の設定と取得
		void setSteerFailsafe(const bool flag) {steer_failsafe_ = flag;}
		bool getSteerFailsafe() const {return steer_failsafe_;}

		//steer_failsafe_変数の設定と取得
		void setPedalFailsafe(const bool flag) {pedal_failsafe_ = flag;}
		bool getPedalFailsafe() const {return pedal_failsafe_;}

		//brake_takeover_変数の設定と取得
		void setBrakeTakeover(const bool flag) {brake_takeover_ = flag;}
		bool getBrakeTakeover() const {return brake_takeover_;}

		//steer_clutch_ban_変数の設定と取得
		void setSteerClutchBan(const bool flag) {steer_clutch_ban_ = flag;}
		bool getSteerClutchBan() const {return steer_clutch_ban_;}

		//pedal_clutch_ban_変数の設定と取得
		void setPedalClutchBan(const bool flag) {pedal_clutch_ban_ = flag;}
		bool getPedalClutchBan() const {return pedal_clutch_ban_;}

		//pedal_clutch_latch_変数の設定と取得
		void setPedalClutchLatch(const bool flag)
		{
			pedal_clutch_latch_ = flag;
			//if(pedal_clutch_latch_ == true) pedal_do_ |= 0x01;
			//else pedal_do_ &= ~(0x01);
		}
		bool getPedalClutchLatch() const {return pedal_clutch_latch_;}

		//CANに送信するステアtargetを履歴に加える
		//[in] steer_cmd:ステアtarget
		//[in] nowtime:古い時間のデータの削除するために、現在を時刻を渡す
		void addSteerCommand(const int16_t steer_cmd, const rclcpp::Time &nowtime)
		{
			steer_command_.insert(steer_command_.begin(), steer_cmd);
			steer_command_time_.insert(steer_command_time_.begin(), nowtime);
			eraseSteerCommand(nowtime);
		}

		//現在の時刻よりも一定時間古いステアコマンド値を削除
		//[in] nowtime:現在時刻
		void eraseSteerCommand(const rclcpp::Time &nowtime)
		{
			for(size_t i=0; i<steer_command_.size(); i++)
			{
				if(nowtime - steer_command_time_[i] > life_cycle_)
				{
					steer_command_.erase(steer_command_.begin() + i, steer_command_.end());
					steer_command_time_.erase(steer_command_time_.begin() + i, steer_command_time_.end());
					break;
				}
			}
			if(steer_command_.size() == 0)
			{
				steer_command_.push_back(STEER_COMMAND_INVALID);
				steer_command_time_.push_back(nowtime);
			}
		}

		//ステア履歴の取得・履歴サイズ・履歴クリア
		int16_t getSteerCommand(const size_t index) const {return steer_command_[index];}
		size_t getSteerCommandSize() const {return steer_command_.size();}
		void clearSteerCommand()
		{
			steer_command_.clear();
			steer_command_time_.clear();
		}

		//CANに送信するペダルtargetを履歴に加える
		//[in] steer_cmd:ペダルtarget
		//[in] nowtime:古い時間のデータの削除するために、現在を時刻を渡す
		void addPedalCommand(const int16_t pedal_cmd, const rclcpp::Time &nowtime)
		{
			pedal_command_.insert(pedal_command_.begin(), pedal_cmd);
			pedal_command_time_.insert(pedal_command_time_.begin(), nowtime);
			erasePedalCommand(nowtime);
		}

		//現在の時刻よりも一定時間古いペダルコマンド値を削除
		//[in] nowtime:現在時刻
		void erasePedalCommand(const rclcpp::Time &nowtime)
		{
			for(size_t i=0; i<pedal_command_.size(); i++)
			{
				if(nowtime - pedal_command_time_[i] > life_cycle_)
				{
					pedal_command_.erase(pedal_command_.begin() + i, pedal_command_.end());
					pedal_command_time_.erase(pedal_command_time_.begin() + i, pedal_command_time_.end());
					break;
				}
			}
			if(pedal_command_.size() == 0)
			{
				pedal_command_.push_back(PEDAL_COMMAND_INVALID);
				pedal_command_time_.push_back(nowtime);
			}
		}

		//ペダル履歴の取得・履歴サイズ・履歴クリア
		int16_t getPedalCommand(const size_t index) const {return pedal_command_[index];}
		size_t getPedalCommandSize() const {return pedal_command_.size();}
		void clearPedalCommand()
		{
			pedal_command_.clear();
			pedal_command_time_.clear();
		}
	};
}
#endif