#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <std_msgs/msg/string.hpp>
#include <traffic_light_signal_msgs/msg/traffic_light_signal_array.hpp>
#include <string>
#include <vector>

class AllTrafficLightSignalsPublisher : public rclcpp::Node
{
public:
  AllTrafficLightSignalsPublisher(const rclcpp::NodeOptions & node_options)
  : rclcpp::Node("all_traffic_light_signals_publisher", node_options)
  {
    pub_dbg_msg_ = this->create_publisher<std_msgs::msg::String>("~/debug/msg", 1);

    std::string message = "AllTrafficLightSignalsPublisher() started";
    std_msgs::msg::String debug_msg;

    received_vector_map_ = false;
    start_publish_ = false;

    sub_vector_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
      "/map/vector_map", rclcpp::QoS{1}.transient_local(),
      std::bind(&AllTrafficLightSignalsPublisher::mapCallback, this, std::placeholders::_1));

    pub_traffic_light_signals_ = this->create_publisher<traffic_light_signal_msgs::msg::TrafficLightSignalArray>(
      "/output/traffic_light_and_signals", rclcpp::QoS(1));

    using namespace std::chrono_literals;
    timer_pub_ = this->create_wall_timer(16ms, std::bind(&AllTrafficLightSignalsPublisher::timer_callback, this));

    debug_msg.data = message;
    pub_dbg_msg_->publish(debug_msg);
  }

private:
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Publisher<traffic_light_signal_msgs::msg::TrafficLightSignalArray>::SharedPtr pub_traffic_light_signals_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_dbg_msg_;
  rclcpp::TimerBase::SharedPtr timer_pub_;

  traffic_light_signal_msgs::msg::TrafficLightSignalArray traffic_light_signals_;
  bool received_vector_map_{false};
  bool start_publish_{false};

  void mapCallback(
    const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg)
  {
    if (received_vector_map_) return;

    std::string message = "mapCallback()";
    std_msgs::msg::String debug_msg;

    lanelet::LaneletMapPtr lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(*input_msg, lanelet_map_ptr);
    lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
    std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
      lanelet::utils::query::autowareTrafficLights(all_lanelets);
    for (auto tl_itr = all_lanelet_traffic_lights.begin(); tl_itr != all_lanelet_traffic_lights.end();
        ++tl_itr) {
      lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

      auto lights = tl->trafficLights();
      for (const auto & light : lights) {
        traffic_light_signal_msgs::msg::TrafficLightSignal light_signal;
        light_signal.traffic_light_id = light.id();
        light_signal.traffic_signal_id = tl->id();
        traffic_light_signals_.array.push_back(light_signal);

        message += (" : traffic_light_id:" + std::to_string(light_signal.traffic_light_id));
        message += (" -> traffic_signal_id:" + std::to_string(light_signal.traffic_signal_id));
      }
    }
    debug_msg.data = message;
    pub_dbg_msg_->publish(debug_msg);
  }

  void timer_callback()
  {
    if (start_publish_) return;

    // Publish traffic_ligt_id and traffic_signal_id
    pub_traffic_light_signals_->publish(traffic_light_signals_);
  }

};


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	std::shared_ptr<AllTrafficLightSignalsPublisher> node = std::make_shared<AllTrafficLightSignalsPublisher>(node_options);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
