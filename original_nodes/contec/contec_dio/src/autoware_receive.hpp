#ifndef CONTEC_AUTOWARE_RECEIVE
#define CONTEC_AUTOWARE_RECEIVE

#include <rclcpp/rclcpp.hpp>
#include <wada_vmc_msgs/msg/can501_20221111.hpp>
#include <wada_vmc_msgs/msg/can502_20221111.hpp>

namespace contec
{
	class AutowareReceive
	{
	private://メンバ変数
		std::vector<rclcpp::Time> blinker_create_time_;//ウィンカーデータを作成した時間の履歴
		std::vector<int8_t> blinker_que_;//ウィンカー情報キュー
		bool handle_clutch_ban_;//ハンドルクラッチ断指令
		bool handle_set_;//ハンドルセット
		bool handle_led_;//ハンドル自動可否LED
		bool handle_failsafe_;//ハンドルフェイルセーフ
		bool pedal_failsafe_;//ペダルフェイルセーフ
		rclcpp::Duration life_cycle_;//データの生存時間
		int8_t no_command_;//ウィンカーコマンドなしを示す数値 ダミーデータ作成に使用
		bool handle_auto_permission_;//ハンドル自動可否
		bool pedal_auto_permission_;//ペダル自動可否
		wada_vmc_msgs::msg::Can501_20221111 can501_;//CAN送信番号0x501
		wada_vmc_msgs::msg::Can502_20221111 can502_;//CAN送信番号0x502

	private://メンバ関数
		//blinker_que_のサイズが0の場合に実行し、ダミーデータを挿入
		//[in] nowtime:現在の時間
		void insertBlinkerEmptyData(const rclcpp::Time &nowtime)
		{
			blinker_create_time_.push_back(nowtime);
			blinker_que_.push_back(no_command_);
		}

	public:
		//コンストラクタ
		//[in] nowtime:インスタンス作成時の時間
		//[in] life_cycle:ウィンカー情報を登録してから、情報を消去する時間
		//[in] no_cmd:ウィンカーコマンドなしを示す数値 ダミーデータ作成に使用
		AutowareReceive(const rclcpp::Time &nowtime, const rclcpp::Duration &life_cycle, const int8_t no_cmd)
			: handle_clutch_ban_(false)
			, handle_set_(false)
			, handle_led_(false)
			, handle_failsafe_(false)
			, pedal_failsafe_(false)
			, life_cycle_(life_cycle)
			, no_command_(no_cmd)
			, handle_auto_permission_(false)
			, pedal_auto_permission_(false)
		{
			insertBlinkerEmptyData(nowtime);
			can501_.stamp = can502_.stamp = nowtime;
		}

	public://AUTOWAREのデータ全体に関する関数
		//データの生存時間を取得
		//返り値:データの生存時間
		rclcpp::Duration getLifeCycle() const
		{
			return life_cycle_;
		}

		//生存時間を過ぎたデータを削除
		//[in] nowtime:現在の時間
		void pops(const rclcpp::Time &nowtime)
		{
			for(size_t i=0; i<blinker_que_.size(); i++)
			{
				if(nowtime - blinker_create_time_[i] > life_cycle_)
				{
					blinker_que_.erase(blinker_que_.begin() + i, blinker_que_.end());
					break;
				}
			}

			if(blinker_que_.size() == 0) insertBlinkerEmptyData(nowtime);//ウィンカー情報がない場合はダミーデータを入れる
		}

	public://ウィンカーの追加・取得・判定の関数
		//ウィンカー情報の取得
		//[in] index:取得したいウィンカー情報キューのインデックス
		//返り値:指定indexのウィンカー情報
		int8_t getBlinker(const size_t index) const
		{
			return blinker_que_[index];
		}

		//新しいウィンカ情報を追加
		//[in] blinker:新しいウィンカー情報
		//[in] nowtime:現在の時間
		void addBlinker(const int8_t blinker, const rclcpp::Time &nowtime)
		{
			blinker_que_.insert(blinker_que_.begin(), blinker);
			blinker_create_time_.insert(blinker_create_time_.begin(), nowtime);
		}

		//ウィンカー情報キューのサイズ取得
		size_t blinkerQueSize() const 
		{
			return blinker_que_.size();
		}

		//ウィンカー履歴の全体の時間を取得
		//返り値:ウィンカー履歴の全体の時間
		rclcpp::Duration blinkerAllTime() const
		{
			return blinker_create_time_[blinker_create_time_.size()-1] - blinker_create_time_[0];
		}

	public://ハンドルクラッチ断線
		bool getHandleClutchBan() const {return handle_clutch_ban_;}
		void setHandleClutchBan(bool val)
		{
			handle_clutch_ban_ = val;
			handle_clutch_ban_ = false;
		}

	public://ハンドルセット
		bool getHandleSet() const {return handle_set_;}
		void setHandleSet(bool val)
		{
			handle_set_ = val;
			handle_set_ = false;
		}

	public://ハンドルLED
		bool getHandleLED() const {return handle_led_;}
		void setHandleLED(bool val) {handle_led_ = val;}

	public://ハンドルフェイルセーフ
		bool getHandleFailsafe() const {return handle_failsafe_;}
		void setHandleFailsafe(bool val) {handle_failsafe_ = val;}

	public://ペダルフェイルセーフ
		bool getPedalFailsafe() const{return pedal_failsafe_;}
		void setPedalFailsafe(bool val) {pedal_failsafe_ = val;}

	public://ハンドル自動可否
		bool getHandleAutoPermission() const {return handle_auto_permission_;}
		void setHandleAutoPermission(bool val) {handle_auto_permission_ = val;}

	public://ペダル自動可否
		bool getPedalAutoPermission() const {return pedal_auto_permission_;}
		void setPedalAutoPermission(bool val) {pedal_auto_permission_ = val;}

	public://can501
		wada_vmc_msgs::msg::Can501_20221111 getCan501() const {return can501_;}
		void setCan501(const wada_vmc_msgs::msg::Can501_20221111 &can) {can501_ = can;}

	public://can502
		wada_vmc_msgs::msg::Can502_20221111 getCan502() const {return can502_;}
		void setCan502(const wada_vmc_msgs::msg::Can502_20221111 &can) {can502_ = can;}
	};
}

#endif