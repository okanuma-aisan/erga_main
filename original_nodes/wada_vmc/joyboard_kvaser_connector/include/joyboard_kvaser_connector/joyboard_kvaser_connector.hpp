//kvaserを介してジョイボードと通信を行うクラスを記述する

#ifndef joyboard_kvaser_connector
#define joyboard_kvaser_connector

#include <canlib.h>
#include <string>

namespace joyboard_kvaser
{

class JoyboardKvaserConnector
{
private:
	bool connect_flag_;//kvaserとconnect出来たらtrue
	canHandle can_handle_;//connectしたkvaserハンドル

	//受信したmessageのステータスを表示
	void printReceive(const unsigned char* const buffer, const long id,
		const unsigned int dlc, const unsigned int flag, const unsigned long time);
public:
	//コンストラクタ
	JoyboardKvaserConnector(const unsigned int serial1, const unsigned int serial2);
	//デストラクタ
	~JoyboardKvaserConnector();

	//kvaserからmessageを受信
	canStatus receive(unsigned char* const buffer, const unsigned long wait_time,
		long &message_id, unsigned int &message_lenght);
	//kvaserにmessageを送信
	canStatus sende(unsigned char* buffer, const long message_id, const unsigned int message_lenght);
	//kvaserにconnectしているかを返す
	bool isConnect();
};
}

#endif