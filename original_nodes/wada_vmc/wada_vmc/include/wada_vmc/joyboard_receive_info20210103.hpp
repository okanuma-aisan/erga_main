#ifndef WADA_VMC_JOY_RECEIVE_INFO
#define WADA_VMC_JOY_RECEIVE_INFO

#include <rclcpp/rclcpp.hpp>
#include <wada_vmc_msgs/msg/can501_20210103.hpp>
#include <wada_vmc_msgs/msg/can502_20210103.hpp>
#include <wada_vmc_msgs/msg/can503_20210103.hpp>

namespace wada_vmc
{
	//joyボード受信情報
	class JoyBoardReceiveInfo
	{
	private://joyボード受信情報
		std::vector<wada_vmc_msgs::msg::Can501_20210103> can501que_;//joyボード情報ID503番の履歴
		std::vector<rclcpp::Time> can501que_time_;//can501que_を登録した時間の履歴
		std::vector<wada_vmc_msgs::msg::Can502_20210103> can502que_;//joyボード情報ID502番の履歴
		std::vector<rclcpp::Time> can502que_time_;//can502que_を登録した時間の履歴
		std::vector<wada_vmc_msgs::msg::Can503_20210103> can503que_;//joyボード情報ID503番の履歴
		std::vector<rclcpp::Time> can503que_time_;//can503que_を登録した時間の履歴

	private://その他メンバ変数
		rclcpp::Duration life_cycle_;//データ生存時間

	public://コンストラクタ
		JoyBoardReceiveInfo(const rclcpp::Time nowtime)
			: life_cycle_(1,0)
		{
			init501(nowtime);
			init502(nowtime);
			init503(nowtime);
		}

		void init501(const rclcpp::Time nowtime)
		{
			wada_vmc_msgs::msg::Can501_20210103 can501;
			can501.stamp = nowtime;
			can501.first_lock = true;
			can501.steer_auto_answer_back = wada_vmc_msgs::msg::Can501_20210103::STEER_MODE_UNESTABLISHED;
			can501.pedal_auto_answer_back = wada_vmc_msgs::msg::Can501_20210103::DRIVE_MODE_UNESTABLISHED;
			can501.steer_mode = wada_vmc_msgs::msg::Can501_20210103::STEER_MODE_UNESTABLISHED;
			can501.drive_mode = wada_vmc_msgs::msg::Can501_20210103::DRIVE_MODE_UNESTABLISHED;
			can501.steer_reply = 0;
			can501.pedal_reply = 0;
			can501.emergency_brake = wada_vmc_msgs::msg::Can501_20210103::EMERGENCY_BRAKE_OFF;
			can501.engine_start = false;
			can501.ignition =false;
			can501.shift_bottom = false;
			can501.accel_intervention = false;
			can501.right_blinker = false;
			can501.left_blinker = false;
			can501.log_start = false;
			can501.auto_drive_ok = false;
			can501.shift_d = true;
			can501.shift_n = false;
			can501.shift_r = false;
			can501.shift_auto = false;
			can501.curse_velocity_plus = false;
			can501.curse_velocity_minus = false;
			can501.distance_between_two_cars = false;
			can501que_.push_back(can501);
			can501que_time_.push_back(nowtime);
		}

		void init502(const rclcpp::Time nowtime)
		{
			wada_vmc_msgs::msg::Can502_20210103 can502;
			can502.stamp = nowtime;
			can502.auto_mode = false;
			can502.pulse_mode = wada_vmc_msgs::msg::Can502_20210103::PULSE_MODE16;
			can502.write_permission_on = false;
			can502.temperature_anomaly = false;
			can502.over_current = false;
			can502.clutch = false;
			can502.brake_lock_set = false;
			can502.motor_disconnection = false;
			can502.power_supply_voltage_drop = false;
			can502.clutch_disconnection = false;
			can502.mecha_potentiometer_disconnection = false;
			can502.joy_disconnection = false;
			can502.handle_pot_average = 0;
			can502.handle_pot_average_v = 0;
			can502.steering_angle = 0;
			can502.steering_angle_deg = 0;
			can502.velocity = 0;
			can502.velocity_kmh = 0;
			can502que_.push_back(can502);
			can502que_time_.push_back(nowtime);
		}

		void init503(const rclcpp::Time nowtime)
		{
			wada_vmc_msgs::msg::Can503_20210103 can503;
			can503.stamp = nowtime;
			can503.auto_mode = false;
			can503.pulse_mode = wada_vmc_msgs::msg::Can503_20210103::PULSE_MODE16;
			can503.abnormal_air_pressure = false;
			can503.write_permission_on = false;
			can503.temperature_anomaly = false;
			can503.over_current = false;
			can503.clutch = false;
			can503.brake_lock_set = false;
			can503.motor_disconnection = false;
			can503.power_supply_voltage_drop = false;
			can503.clutch_disconnection = false;
			can503.mecha_potentiometer_disconnection = false;
			can503.joy_disconnection = false;
			can503.pedal_command_voltage = 0;
			can503.pedal_command_voltage_v = 0;
			can503.pedal_displacement = 0;
			can503.engine_rpm = 0;
			can503que_.push_back(can503);
			can503que_time_.push_back(nowtime);
		}

	public:
		//CAN501受信情報を履歴に加える
		//[in] can501:CAN501受信情報
		//[in] nowtime:古い時間のデータの削除するために、現在を時刻を渡す
		void add501(const wada_vmc_msgs::msg::Can501_20210103 &can501, const rclcpp::Time &nowtime)
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
		wada_vmc_msgs::msg::Can501_20210103 get501(const size_t index) const {return can501que_[index];}
		size_t get501Size() const {return can501que_.size();}
		void clear501()
		{
			can501que_.clear();
			can501que_time_.clear();
		}

		//CAN502受信情報を履歴に加える
		//[in] can502:CAN502受信情報
		//[in] nowtime:古い時間のデータの削除するために、現在を時刻を渡す
		void add502(const wada_vmc_msgs::msg::Can502_20210103 &can502, const rclcpp::Time &nowtime)
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
		wada_vmc_msgs::msg::Can502_20210103 get502(const size_t index) const {return can502que_[index];}
		size_t get502Size() const {return can502que_.size();}
		void clear502()
		{
			can502que_.clear();
			can502que_time_.clear();
		}

		//CAN503受信情報を履歴に加える
		//[in] can503:CAN503受信情報
		//[in] nowtime:古い時間のデータの削除するために、現在を時刻を渡す
		void add503(const wada_vmc_msgs::msg::Can503_20210103 &can503, const rclcpp::Time &nowtime)
		{
			can503que_.insert(can503que_.begin(), can503);
			can503que_time_.insert(can503que_time_.begin(), nowtime);
			for(size_t i=0; i<can503que_.size(); i++)
			{
				if(nowtime - can503que_time_[i] > life_cycle_)
				{
					can503que_.erase(can503que_.begin() + i, can503que_.end());
					can503que_time_.erase(can503que_time_.begin() + i, can503que_time_.end());
					break;
				}
			}
		}

		//can503履歴の取得・履歴サイズ・履歴クリア
		wada_vmc_msgs::msg::Can503_20210103 get503(const size_t index) const {return can503que_[index];}
		size_t get503Size() const {return can503que_.size();}
		void clear503()
		{
			can503que_.clear();
			can503que_time_.clear();
		}
	};
}

#endif