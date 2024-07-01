import rclpy
from rclpy.node import Node

from tier4_perception_msgs.msg import TrafficLightRoiArray, TrafficLightRoi
from std_msgs.msg import String


DEFAULT_INTERVAL_TIME = (1.0 / 30)
DEFAULT_FRAME_ID = "camera6_optical_link"
DEFAULT_TRAFFIC_LIGHT_ID = 2180
DEFAULT_ROI = { 'x':0, 'y':0, 'w':1920, 'h':1280 }

class DummyExpectRoughRoisPub(Node):
    def __init__(self):
        super().__init__('dummy_expect_rough_rois_pub')
        print("start dummy_expect_rough_rois_pub")
        # Get parameters
        self.declare_parameter('interval_time', DEFAULT_INTERVAL_TIME)
        self.interval_time = self.get_parameter('interval_time').get_parameter_value().double_value
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.declare_parameter('traffic_light_id', DEFAULT_TRAFFIC_LIGHT_ID)
        self.traffic_light_id = self.get_parameter('traffic_light_id').get_parameter_value().integer_value
        self.declare_parameter('x_offset', DEFAULT_ROI['x'])
        self.x_offset = self.get_parameter('x_offset').get_parameter_value().integer_value
        self.declare_parameter('y_offset', DEFAULT_ROI['y'])
        self.y_offset = self.get_parameter('y_offset').get_parameter_value().integer_value
        self.declare_parameter('width', DEFAULT_ROI['w'])
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.declare_parameter('height', DEFAULT_ROI['h'])
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        # Create Publishers
        self.expect_rois_pub = self.create_publisher(TrafficLightRoiArray, "/output/expect_rois", 10)
        self.rough_rois_pub = self.create_publisher(TrafficLightRoiArray, "/output/rough_rois", 10)
        self.debug_msg_pub = self.create_publisher(String, '/output/debug/msg', 20)
        # Interval timer
        self.signal_out_timer = self.create_timer(self.interval_time, self.rois_out)

    def rois_out(self):
        traffic_light_rois = TrafficLightRoiArray()
        traffic_light_rois.header.stamp = self.get_clock().now().to_msg()
        traffic_light_rois.header.frame_id = self.frame_id
        traffic_light_roi = TrafficLightRoi()
        traffic_light_roi.traffic_light_id = self.traffic_light_id
        traffic_light_roi.roi.x_offset = self.x_offset
        traffic_light_roi.roi.y_offset = self.y_offset
        traffic_light_roi.roi.width = self.width
        traffic_light_roi.roi.height = self.height
        traffic_light_rois.rois.append(traffic_light_roi)
        self.expect_rois_pub.publish(traffic_light_rois)
        self.rough_rois_pub.publish(traffic_light_rois)


def main(args=None):
    rclpy.init(args=args)
    main_node = DummyExpectRoughRoisPub()
    try:
        rclpy.spin(main_node)
    except KeyboardInterrupt:
        print("Shutting down")

    main_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
