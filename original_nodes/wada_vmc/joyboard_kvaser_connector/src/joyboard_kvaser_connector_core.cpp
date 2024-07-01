//kvaserを介してジョイボードと通信を行うクラスを記述する

#include <joyboard_kvaser_connector/joyboard_kvaser_connector.hpp>
#include <iostream>

//kvaserを介してジョイボードと通信を行うプログラムのnamespace
namespace joyboard_kvaser
{
	//コンストラクタ
	//[in] serial1:kvaserシリアル番号前半
	//[in] serial2:kvaserシリアル番号後半
	JoyboardKvaserConnector::JoyboardKvaserConnector(const unsigned int serial1, const unsigned int serial2)
		: connect_flag_(false)
	{
		canInitializeLibrary();

		int chan_count = 0;
		canStatus stat = canGetNumberOfChannels(&chan_count);
		if (stat != canStatus::canOK)
		{
			std::cerr<< "error : canGetNumberOfChannels";
			return;
		}

		int open_channel = INT32_MAX;
		for(int ch=0; ch<chan_count; ch++)
		{
			unsigned int serial[2];
			stat = canGetChannelData(ch, canCHANNELDATA_CARD_SERIAL_NO,
            	                 &serial, sizeof(serial));
			if (stat != canStatus::canOK)
			{
				std::cerr<< "error : canGetChannelData : " << ch;
				return;
			}

			std::cerr << serial1 << "=" << serial[0] << std::endl;
			std::cerr << serial2 << "=" << serial[1] << std::endl;
			if(serial[0] == serial1 && serial[1] == serial2)
			{
				open_channel = ch;
				break;
			}
		}

		if(open_channel != INT32_MAX)
		{
			can_handle_ = canOpenChannel(open_channel, 0);//canOPEN_CAN_FD);
			if(can_handle_ < 0)
			{
				std::cerr<< "error : canOpenChannel : " << open_channel;
				return;
			}
			stat = canSetBusParams(can_handle_, canBITRATE_500K, 0, 0, 0, 0, 0);
			if(stat != canStatus::canOK)
			{
				std::cerr<< "error : canSetBusParams : " << open_channel;
				return;
			}
			stat = canBusOn(can_handle_);
			if(stat != canStatus::canOK)
			{
				std::cerr<< "error : canSetBusParams : " << open_channel;
				return;
			}

			std::cout<< "open channel is " << open_channel << " : serial1," << serial1 << " : serial2," << serial2 << std::endl;
			connect_flag_ = true;
		}
		else
		{
			std::cerr<< "nothing serial : serial1," << serial1 << " : serial2," << serial2 << std::endl; 
		}
	}

	//デストラクタ
	JoyboardKvaserConnector::~JoyboardKvaserConnector()
	{
		if(can_handle_ == canStatus::canOK)
			canBusOff(can_handle_);
	}

	//kvaserからmessageを受信
	//[in] buffer:messageを受信するバッファ
	//[in] wait_time:time out時間
	//[out] message_id:受信message ID
	//[out] message_lenght:受信したmessageの長さ
	canStatus JoyboardKvaserConnector::receive(unsigned char* const buffer, const unsigned long wait_time,
		long &message_id, unsigned int &message_lenght)
	{
		long id;
		unsigned int dlc, flag;
		unsigned long time;

		canStatus stat = canReadWait(can_handle_, &id, buffer, &dlc, &flag, &time, wait_time);
		message_lenght = dlc;
		message_id = id;
		if(stat != canStatus::canOK)
		{
			std::cerr<< "error : canReadWait : " << stat << "," << canStatus::canOK << std::endl;
		}
		//else this->printReceive(buffer, id, dlc, flag, time);
		return stat;
	}

	//受信したmessageのステータスを表示
	//[in] buffer:受信したmessage
	//[in] id:受信したmessageのID
	//[in] dlc:受信したmessageの長さ
	//[in] flag:受信したmessageの受信フラグ
	//[in] time:受信にかかった時間
	void JoyboardKvaserConnector::printReceive(const unsigned char* const buffer, const long id,
		const unsigned int dlc, const unsigned int flag, const unsigned long time)
	{
		printf("id:%lx dlc:%u data: ", id, dlc);
		for (unsigned int j = 0; j < dlc; j++) {
		  printf("%2.2x ", buffer[j]);
		}
		printf(" flags:0x%x time:%lu\n", flag, time);
		fflush(stdout);
	}

	//kvaserにmessageを送信
	//[in] buffer:送信するmessageのバッファ
	//[in] message_id:送信message ID
	//[in] message_lenght:送信するmessageの長さ
	canStatus JoyboardKvaserConnector::sende(unsigned char* buffer, const long message_id, const unsigned int message_lenght)
	{
		canStatus res = canWrite(can_handle_, message_id, reinterpret_cast<void*>(buffer), message_lenght, 0);
		if(canStatus::canOK != res) return res;

		return canStatus::canOK;
	}

	//kvaserにconnectしているかを返す
	bool JoyboardKvaserConnector::isConnect() {return connect_flag_;}
}