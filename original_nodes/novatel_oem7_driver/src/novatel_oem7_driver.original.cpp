#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class NovatelOem7Driver : public rclcpp::Node
{
private:
	std::string serial_;//シリアルデバイス
	int baud_;//baudレート
	int file_descriptor_; //ファイルディスクリプタ

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_sentence_;
public:
	NovatelOem7Driver()
		: Node("novatel_oem7_driver")
		, file_descriptor_(-1)
	{
		serial_ = declare_parameter("serial", "/dev/ttyUSB0");
		baud_ = declare_parameter("baud", 115200);

		pub_sentence_ = create_publisher<std_msgs::msg::String>("sentence", rclcpp::SensorDataQoS());
	}

	//リシアルポートに接続
	bool setupSerialDevice()
	{
		int baud_macro;
		if(baud_ == 110) baud_macro = B110;
		else if(baud_ == 300) baud_macro = B300;
		else if(baud_ == 1200) baud_macro = B1200;
		else if(baud_ == 2400) baud_macro = B2400;
		else if(baud_ == 4800) baud_macro = B4800;
		else if(baud_ == 9600) baud_macro = B9600;
		else if(baud_ == 19200) baud_macro = B19200;
		else if(baud_ == 38400) baud_macro = B38400;
		else if(baud_ == 57600) baud_macro = B57600;
		else if(baud_ == 115200) baud_macro = B115200;
		else if(baud_ == 230400) baud_macro = B230400;
		else
		{
			RCLCPP_ERROR(this->get_logger(), "The baud rate can not be specified.");
			return false;
		}

		struct termios tio;                 // シリアル通信設定

		/*std::string command("sudo chmod 666 ");  //デバイスの権限を現在のアカウントで使用可能にする
		command  += serial_;
		while( 0 != system(command.c_str()) )
		{
			RCLCPP_WARN(this->get_logger(), "chmod error\n");
			return false;
		}*/

		file_descriptor_ = open(serial_.c_str(), O_RDWR);     // デバイスをオープンする
		if (file_descriptor_ < 0) {
			RCLCPP_WARN(this->get_logger(), "open error\n");
			return false;
		}

		memset(&tio,0,sizeof(tio));
		tio.c_cflag += CREAD;               // 受信有効
		tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
		tio.c_cflag += CS8;                 // データビット:8bit
		tio.c_cflag += 0;                   // ストップビット:1bit
		tio.c_cflag += PARENB;              // パリティ:None
		tio.c_iflag = IGNPAR;               // パリティエラー無視
		tio.c_lflag = 0;                    // non-canonical
		tio.c_cc[VTIME] = 1;                // read time out 100ms 
		tio.c_cc[VMIN] = 0;                 // １文字受信

		cfsetispeed( &tio, baud_macro );      // ボーレート設定
		cfsetospeed( &tio, baud_macro );

		tcflush( file_descriptor_, TCIFLUSH );
		tcsetattr( file_descriptor_, TCSANOW, &tio );     // デバイスに設定を行う
		ioctl(file_descriptor_, TCSETS, &tio);            // ポートの設定を有効にする

		return true;
	}

	void receiveData()
	{
		const std::string START_STRING1("#");//novatel oem7 センテンスの開始文字列
		const std::string START_STRING2("$");//novatel oem7 センテンスの開始文字列
		const std::string TERMINATE_STRING("\r\n"); //novatel oem7 センテンスの終了文字列

		// receive nmea string data
		//std::string nmea_string= "";
		//std_msgs::msg::String str_message;

		const size_t BUF_SIZE = 300;
		char buf[BUF_SIZE];
		int len = read(file_descriptor_, buf, sizeof(buf));
		//RCLCPP_INFO(this->get_logger(), "len,%d", len);
		//RCLCPP_INFO(this->get_logger(), "sen,%s\n", buf);

		if(START_STRING1.front() != buf[0] && START_STRING2.front() != buf[0])
		{
			RCLCPP_WARN(this->get_logger(), "WARN : not start string \'#\'");
			return;
		}

		//std::string::size_type end_pos = nmea_string.find(TERMINATE_STRING);
		//if(end_pos == std::string::npos)
		if(buf[len-2] != TERMINATE_STRING[0] || buf[len-1] != TERMINATE_STRING[1])
		{
			RCLCPP_WARN(this->get_logger(), "WARN : not end string");
			return;
		}

		buf[len-2] = '\0';
		//RCLCPP_INFO(this->get_logger(), "sen,%s\n", buf);

		//std::string nmea_string(buf);
		std_msgs::msg::String nmea_pub_string;
		nmea_pub_string.data.append(buf);
		pub_sentence_->publish(nmea_pub_string);
		/*while(canReceiveData)
		{
			
			unsigned char buf;
			int len = read(file_descriptor_, &buf, sizeof(buf));
			RCLCPP_INFO(this->get_logger(), "len,%d", len);

			if( len < 0 )
			{
				// read fatal error
				str_message.data = std::string("read fatal error errno=") + std::to_string(errno);
				pub_receive_error_->publish(str_message);
				break;
			}
			else if( len == 0 )
			{
				// read time out
				rclcpp::Time timeout = this->now();
				str_message.data = std::string("read time out ") + std::to_string(timeout.seconds());
				pub_receive_error_->publish(str_message);
				continue;
			}

			// 開始文字
			if(START_STRING.front() == buf)
			{
				// 受信未完了のデータチェック
				if( nmea_string.length() != 0)
				{
					str_message.data = nmea_string;
					//pub_sentence_->publish(str_message);
				}

				// 開始データをセット
				nmea_string = START_STRING;
				continue;
			}
			// 開始文字以外 → 開始文字受信前のデータ受信確認
			else if(START_STRING.front() != nmea_string.front())
			{

				str_message.data = buf;
				//ROS_WARN("CHAR=%c",buf);
				//pub_sentence_->publish((str_message));

				// 受信中データ破棄
				nmea_string = "";
				continue;
			}
			// 開始文字受信後のデータ受信
			else
			{
				nmea_string += buf;

				// 終端文字確認
				std::string::size_type pos = nmea_string.find(TERMINATE_STRING);
				if(pos == std::string::npos)
				{
					// terminate data receive no yet. receive continue.
					continue;
				}
				
				// 受信完了
				//std::string rep = replaceOtherStr(nmea_string, "\"", "'");
				//rep = replaceOtherStr(rep, TERMINATE_STRING.c_str(), "\0");
				//publish(rep.c_str(), rep.size());
				pub_sentence_->publish((str_message));

				nmea_string = "";
			}
		}*/

		return;
	}

	void fileDescriptorClose()
	{
		close(file_descriptor_);
		file_descriptor_ = -1;
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<NovatelOem7Driver> driver = std::make_shared<NovatelOem7Driver>();

	while(rclcpp::ok())
	{
		if(driver->setupSerialDevice() == true)
		{
			driver->receiveData();
			driver->fileDescriptorClose();
		}
		rclcpp::spin_some(driver);
	}
	rclcpp::shutdown();
	return 0;
}