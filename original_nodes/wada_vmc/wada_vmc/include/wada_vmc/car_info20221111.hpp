#ifndef WADA_VMC_CAR_INFO
#define WADA_VMC_CAR_INFO

#include <stdint.h>

namespace wada_vmc
{
	//車両固有情報
	class CarInfo
	{
	public:
		int16_t steer_input_center_;//ステアコマンド中央値
		int16_t steer_input_left_max_;//ステアコマンド最大値(左回転)
		int16_t steer_input_right_min_;//ステアコマンド最小値(右回転)
		int16_t pedal_input_center_;//ペダルコマンド中央値
		int16_t pedal_input_brake_max_;//ペダルコマンド最大値(ブレーキ)
		int16_t pedal_input_accel_min_;//ペダルコマンド最小値(アクセル)
		/*bool pedal_auto_permission_;//ペダル自動可否
		bool pedal_failsafe_;//ペダルフェイルセーフ  trueでフェイルセイフ
		bool breke_takeover_;//ブレーキのテイクオーバー保持非保持切り替え（falseで状態保持）
		bool pedal_clutch_;//ペダルクラッチ入切信号 trueならclutchが切れるかも？(未確認)
		bool handle_auto_permission_;//ハンドル自動可否
		bool handle_failsafe_;//ハンドルフェイルセーフ  trueでフェイルセイフ
		bool handle_clutch_;//ハンドルクラッチ入切信号 trueならclutchが切れるかも？(未確認)*/
		double accel_scale_factor_;//アクセルスケールファクター
		double brake_scale_factor_;//ブレーキスケールファクター
		double cmd_brake_scale_factor_;//ブレーキの直値変換用ファクター

		CarInfo()
			/*: pedal_auto_permission_(true)
			, pedal_failsafe_(false)
			, breke_takeover_(false)
			: pedal_clutch_(false)
			, handle_auto_permission_(true)
			, handle_failsafe_(false)
			, handle_clutch_(false)*/
		{

		}
	};
}

#endif
