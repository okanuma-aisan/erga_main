#ifndef WADA_VMC_CAR_INFO
#define WADA_VMC_CAR_INFO

#include <stdint.h>

namespace wada_vmc
{
	//車両固有情報
	class CarInfo
	{
	public:
		int16_t steer_input_left_max_;//ステアコマンド最大値(左回転)
		int16_t steer_input_right_min_;//ステアコマンド最小値(右回転)
		int16_t pedal_input_brake_min_;//ペダルコマンド最小値(ブレーキ)
		int16_t pedal_input_accel_max_;//ペダルコマンド最大値(アクセル)

		CarInfo()
		{

		}
	};
}

#endif
