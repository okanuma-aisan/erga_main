#ifndef WADA_VMC_JOYBOARD_SEND_INFO
#define WADA_VMC_JOYBOARD_SEND_INFO

#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>

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
		std::vector<uint8_t> blinker_command_;//現在のウィンカー指令の履歴
		std::vector<rclcpp::Time> blinker_command_time_;//blinker_command_を登録した時間の履歴
		uint8_t prev_blinker_trigger_;//前回のウィンカー指令
		bool steer_failsafe_;//ステア自動可否
		bool pedal_failsafe_;//ステア自動可否
		bool failsafe_light_;//フェイルセーフライト
		std::vector<int16_t> steer_command_;//現在のステア入力指令の履歴
		std::vector<rclcpp::Time> steer_command_time_;//steer_command_を登録した時間の履歴
		std::vector<int16_t> pedal_command_;//現在のペダル入力指令の履歴
		std::vector<rclcpp::Time> pedal_command_time_;//pedal_command_を登録した時間の履歴

	private://その他メンバ変数
		rclcpp::Duration life_cycle_;//データ生存時間

	public:
		JoyBoardSendInfo(const rclcpp::Time  &nowtime)
			: operation_autoware_(true)
			, operation_blnker_(true)
			, steer_auto_(true)
			, pedal_auto_(true)
			, prev_blinker_trigger_(autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND)
			, steer_failsafe_(false)
			, pedal_failsafe_(false)
			, failsafe_light_(false)
			, life_cycle_(1,0)
		{
			blinker_command_.push_back(autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND);
			blinker_command_time_.push_back(nowtime);
			steer_command_.push_back(STEER_COMMAND_INVALID);
			steer_command_time_.push_back(nowtime);
			pedal_command_.push_back(PEDAL_COMMAND_INVALID);
			pedal_command_time_.push_back(nowtime);
			autoware_auto_control_msgs::msg::AckermannControlCommand cmd;
			cmd.longitudinal.speed = CONTROL_COMMAND_INVALID;
			cmd.stamp = nowtime;
			//control_cmd_.push_back(cmd);
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

		//AUTOWAREからのウィンカー指令を履歴に加える
		//[in] blinker:ウィンカー指令
		//[in] nowtime:古い時間のデータの削除するために、現在を時刻を渡す
		void addBlinkerCommand(const uint8_t blinker, const rclcpp::Time &nowtime)
		{
			blinker_command_.insert(blinker_command_.begin(), blinker);
			blinker_command_time_.insert(blinker_command_time_.begin(), nowtime);
			eraseBlinkerCommand(nowtime);
		}

		//現在の時刻よりも一定時間古いウィンカー情報を削除
		//[in] nowtime:現在時刻
		void eraseBlinkerCommand(const rclcpp::Time &nowtime)
		{
			for(size_t i=0; i<blinker_command_.size(); i++)
			{
				if(nowtime - blinker_command_time_[i] > life_cycle_)
				{
					blinker_command_.erase(blinker_command_.begin() + i, blinker_command_.end());
					blinker_command_time_.erase(blinker_command_time_.begin() + i, blinker_command_time_.end());
					break;
				}
			}
			if(blinker_command_.size() == 0)
			{
				blinker_command_.push_back(autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND);
				blinker_command_time_.push_back(nowtime);
			}
		}

		//ウィンカー履歴からCANに送信するウィンカー指令を取得
		uint8_t getBlinkerTrigger()
		{
			uint8_t bc;
			if(blinker_command_.size() == 1) bc = blinker_command_[0];
			else if(blinker_command_[0] != blinker_command_[1]) bc = blinker_command_[0];
			else bc = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND;

			uint8_t ret = (bc == prev_blinker_trigger_) ? autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND : bc;
			prev_blinker_trigger_ = bc;
			return ret;
		}

		//ウィンカー履歴の取得・履歴サイズ・履歴クリア
		uint8_t getBlinkerCommand(const size_t index) const {return blinker_command_[index];}
		size_t getBlinkerCommandSize() const {return blinker_command_.size();}
		void clearBlinkerCommand()
		{
			blinker_command_.clear();
			blinker_command_time_.clear();
		}

		//steer_auto_permission_変数の設定と取得
		void setSteerFailSafe(const bool flag) {steer_failsafe_ = flag;}
		bool getSteerFailSafe() const {return steer_failsafe_;}

		//pedal_auto_permission_変数の設定と取得
		void setPedalFailSafe(const bool flag) {pedal_failsafe_ = flag;}
		bool getPedalFailSafe() const {return pedal_failsafe_;}

		//failsafe_変数の設定と取得
		void setFailsafeLight(const bool flag) {failsafe_light_ = flag;}
		bool getFailsafeLight() const {return failsafe_light_;}

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