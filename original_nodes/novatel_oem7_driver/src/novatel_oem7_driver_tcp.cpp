#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <std_msgs/msg/string.hpp>

class NovatelOem7DriverTcp : public rclcpp::Node
{
private:
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_sentence_;

	std::unique_ptr<boost::asio::io_service> io_;
	std::unique_ptr<boost::asio::ip::tcp::socket> sock_;
	boost::asio::ip::tcp::endpoint endpoint_;
	bool connect_ok_;
public:
	NovatelOem7DriverTcp()
		: Node("novatel_oem7_driver_tcp")
		, connect_ok_(false)
	{
		pub_sentence_ = this->create_publisher<std_msgs::msg::String>("sentence", rclcpp::SensorDataQoS());

		std::string ip = declare_parameter("ip", "192.168.1.150");
		int port = declare_parameter("port", 3001);

		// connect server PortNo=3001
		//boost::asio::io_service io;
		io_ = std::make_unique<boost::asio::io_service>();
		sock_ = std::make_unique<boost::asio::ip::tcp::socket>(*io_);
		endpoint_ = boost::asio::ip::tcp::endpoint{boost::asio::ip::address::from_string(ip), port};
		boost::system::error_code error;
		sock_->connect(endpoint_, error);

		if(error) RCLCPP_ERROR(this->get_logger(), "error : not connect");
		else
		{
			RCLCPP_INFO(this->get_logger(), "connect OK");
			connect_ok_ = true;
		}
	}

	void receiveData()
	{
		boost::system::error_code error;
		boost::asio::streambuf readbuf;
		boost::asio::read_until(*sock_, readbuf, "\r\n", error);

		std::string all_str = boost::asio::buffer_cast<const char*>(readbuf.data());
		std::vector<std::string> nmea_array;
		boost::algorithm::split(nmea_array, all_str, boost::is_any_of("\n"), boost::algorithm::token_compress_on);//boost::is_any_of("\n"));

		for(std::string nmea_parts : nmea_array)
		{
			std::cout << nmea_parts << std::endl;
		}

		for(std::string nmea_parts : nmea_array)
		{
			std::string::iterator begin = nmea_parts.begin();
			std::string::iterator end = nmea_parts.end();
			std::cout << *(end-1) << std::endl;
			if(*begin == '#' && *(end-1) == '\r')
			{//OK
				//publish(nmea_parts.c_str(), nmea_parts.size());
				std_msgs::msg::String sentence;
				sentence.data = nmea_parts;
				pub_sentence_->publish(sentence);
				break;
			}
			else if(*begin == '$' && *(end-1) == '\r')
			{//OK
				//publish(nmea_parts.c_str(), nmea_parts.size());
				std_msgs::msg::String sentence;
				sentence.data = nmea_parts;
				pub_sentence_->publish(sentence);
				break;
			}
			else
			{
				/*std_msgs::String str_message;
				str_message.data = nmea_parts;
				nmea_string_pub.publish(str_message);*/
			}
		}
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<NovatelOem7DriverTcp> driver = std::make_shared<NovatelOem7DriverTcp>();

	while(rclcpp::ok())
	{
		/*if(driver->setupSerialDevice() == true)
		{
			driver->receiveData();
			driver->fileDescriptorClose();
		}*/

		driver->receiveData();
		//rclcpp::spin_some(driver);
	}
	rclcpp::shutdown();
	return 0;
}