#ifndef WADA_VMC_JOY_RECEIVE_INFO
#define WADA_VMC_JOY_RECEIVE_INFO

#include <rclcpp/rclcpp.hpp>
#include <wada_vmc_msgs/msg/can501_20221111.hpp>
#include <wada_vmc_msgs/msg/can502_20221111.hpp>

namespace wada_vmc
{
	//joyボード受信情報
	class JoyBoardReceiveInfo
	{
	private://joyボード受信情報
		std::vector<wada_vmc_msgs::msg::Can501_20221111> can501que_;//joyボード情報ID503番の履歴
		std::vector<rclcpp::Time> can501que_time_;//can501que_を登録した時間の履歴
		std::vector<wada_vmc_msgs::msg::Can502_20221111> can502que_;//joyボード情報ID502番の履歴
		std::vector<rclcpp::Time> can502que_time_;//can502que_を登録した時間の履歴

	private://その他メンバ変数
		rclcpp::Duration life_cycle_;//データ生存時間

	public://コンストラクタ
		JoyBoardReceiveInfo(const rclcpp::Time nowtime)
			: life_cycle_(1,0)
		{
			init501(nowtime);
			init502(nowtime);
		}

		void init501(const rclcpp::Time nowtime)
		{
			wada_vmc_msgs::msg::Can501_20221111 can501;
			can501.stamp = nowtime;
			can501.drive_mode = wada_vmc_msgs::msg::Can501_20221111::DRIVE_MODE_UNESTABLISHED;
			can501.speed_kmh = 0;
			can501.pedal_do = 0;
			can501.pedal_pot_sub_v = 1024;
			can501.pedal_di = 0;
			can501.pedal_pot_main_v = 1024;
			can501.auto_mode = 0;
			can501.pulse_mode = wada_vmc_msgs::msg::Can501_20221111::PULSE_MODE16;
			can501.abnormal_air_pressure = 0;
			can501.write_permission_on = 0;
			can501.temperature_anomaly = 0;
			can501.over_current = 0;
			can501.clutch = 0;
			can501.brake_lock_set = 0;
			can501.motor_disconnection = 0;
			can501.power_supply_voltage_drop = 0;
			can501.mecha_potentiometer_disconnection = 0;
			can501.joy_disconnection = 0;
			can501que_.push_back(can501);
			can501que_time_.push_back(nowtime);
		}

		void init502(const rclcpp::Time nowtime)
		{
			wada_vmc_msgs::msg::Can502_20221111 can502;
			can502.stamp = nowtime;
			can502.handle_mode = wada_vmc_msgs::msg::Can502_20221111::HANDLE_MODE_UNESTABLISHED;
			can502.engin_rmp = 0;
			can502.handle_do = 0;
			can502.handle_pot_sub_v = 1024;
			can502.handle_di = 0;
			can502.handle_pot_main_v = 1024;
			can502.auto_mode = 0;
			can502.pulse_mode = wada_vmc_msgs::msg::Can502_20221111::PULSE_MODE16;
			can502.write_permission_on = 0;
			can502.temperature_anomaly = 0;
			can502.over_current = 0;
			can502.clutch = 0;
			can502.brake_lock_set = 0;
			can502.motor_disconnection = 0;
			can502.power_supply_voltage_drop = 0;
			can502.clutch_disconnection = 0;
			can502.mecha_potentiometer_disconnection = 0;
			can502.joy_disconnection = 0;
			can502que_.push_back(can502);
			can502que_time_.push_back(nowtime);
		}

	public:
		//CAN501受信情報を履歴に加える
		//[in] can501:CAN501受信情報
		//[in] nowtime:古い時間のデータの削除するために、現在を時刻を渡す
		void add501(const wada_vmc_msgs::msg::Can501_20221111 &can501, const rclcpp::Time &nowtime)
		{
			can501que_.insert(can501que_.begin(), can501);
			can501que_time_.insert(can501que_time_.begin(), nowtime);
			for(size_t i=0; i<can501que_.size(); i++)
			{
				if(nowtime - can501que_time_[i] > life_cycle_)
				{
					can501que_.erase(can501que_.begin() + i, can501que_.end());
					can501que_time_.erase(can501que_time_.begin() + i, can501que_time_.end());
					break;
				}
			}
		}

		//can501履歴の取得・履歴サイズ・履歴クリア
		wada_vmc_msgs::msg::Can501_20221111 get501(const size_t index) const {return can501que_[index];}
		size_t get501Size() const {return can501que_.size();}
		void clear501()
		{
			can501que_.clear();
			can501que_time_.clear();
		}

		//CAN502受信情報を履歴に加える
		//[in] can502:CAN502受信情報
		//[in] nowtime:古い時間のデータの削除するために、現在を時刻を渡す
		void add502(const wada_vmc_msgs::msg::Can502_20221111 &can502, const rclcpp::Time &nowtime)
		{
			can502que_.insert(can502que_.begin(), can502);
			can502que_time_.insert(can502que_time_.begin(), nowtime);
			for(size_t i=0; i<can502que_.size(); i++)
			{
				if(nowtime - can502que_time_[i] > life_cycle_)
				{
					can502que_.erase(can502que_.begin() + i, can502que_.end());
					can502que_time_.erase(can502que_time_.begin() + i, can502que_time_.end());
					break;
				}
			}
		}

		//can502履歴の取得・履歴サイズ・履歴クリア
		wada_vmc_msgs::msg::Can502_20221111 get502(const size_t index) const {return can502que_[index];}
		size_t get502Size() const {return can502que_.size();}
		void clear502()
		{
			can502que_.clear();
			can502que_time_.clear();
		}
	};
}

#endif