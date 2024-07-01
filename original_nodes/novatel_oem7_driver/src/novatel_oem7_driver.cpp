#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define LINE_BUFFER_SIZE 1024
class NovatelOem7Driver : public rclcpp::Node
{
private:
	std::string serial_;//シリアルデバイス
	int baud_;//baudレート
	int file_descriptor_; //ファイルディスクリプタ

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_sentence_;
	//char line_buffer[LINE_BUFFER_SIZE];
	std::array<char, LINE_BUFFER_SIZE> line_buffer{};
	long unsigned int line_buffer_idx;
	bool has_published;
	bool show_error_;
public:
	NovatelOem7Driver()
		: Node("novatel_oem7_driver")
		, file_descriptor_(-1)
		, show_error_(true)
	{
		serial_ = declare_parameter("serial", "/dev/ttyUSB0");
		baud_ = declare_parameter("baud", 115200);

		pub_sentence_ = create_publisher<std_msgs::msg::String>("sentence", rclcpp::SensorDataQoS());
		
		line_buffer_idx = 0;
		has_published = true;
		//show_error_ = true;
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
			if (show_error_ == false) return false; // エラーを表示しないようにする
			RCLCPP_WARN(this->get_logger(), "open error\n");
			show_error_ = false; // 次の段階でエラーを表示しないようにする
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
		/* Modified by Miyajima (7/29/2023) */
		
		// input buffer is parsed, complete lines get published
		// if an incomplete line is in the buffer, its kept in line_buffer until the next input buffer chunk

		// larger input buffer = lower read latency, longer processing time
		// smaller input buffer = higher read latency, lower processing time
		static const size_t INPUT_BUFFER_SIZE = 1024;
		char input_buffer[INPUT_BUFFER_SIZE];
		int len = read(file_descriptor_, input_buffer, INPUT_BUFFER_SIZE);
		if (len < 0) {
			RCLCPP_WARN(this->get_logger(), "File descriptor read error.");
			return;
		}

		if (len == 0) {
			RCLCPP_WARN(this->get_logger(), "EOF Received.");
			return;
		}
		
		for (long unsigned int idx = 0; idx < INPUT_BUFFER_SIZE; idx++) {
			//RCLCPP_INFO(this->get_logger(), "%ld", idx);
			if (input_buffer[idx] == '#' || input_buffer[idx] == '$') { // start of valid string
				has_published = false;
				line_buffer_idx = 1;
				line_buffer[0] = input_buffer[idx];
				continue;
			}

			if (input_buffer[idx] == '\r' || input_buffer[idx] == '\n') { // end of valid string
				if (has_published) continue;
				has_published = true;
				line_buffer[line_buffer_idx] = '\0';
				std_msgs::msg::String nmea_pub_string;
				nmea_pub_string.data.append(line_buffer.data());
				pub_sentence_->publish(nmea_pub_string);
				continue;
			}

			if (has_published) continue;
			line_buffer[line_buffer_idx] = input_buffer[idx];
			line_buffer_idx++;
		}
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

	// Wait for device setup to finish
	RCLCPP_INFO(driver->get_logger(), "Attempting to set up device ...");
	while ( rclcpp::ok() && driver->setupSerialDevice() == false ) {
		rclcpp::spin_some(driver);
	}

	// Receive data
	RCLCPP_INFO(driver->get_logger(), "Starting receive of data ...");
	while( rclcpp::ok() ) {
		driver->receiveData();
		rclcpp::spin_some(driver);
	}

	driver->fileDescriptorClose();
	rclcpp::shutdown();
	return 0;
}
