import rclpy
from rclpy.node import Node, Clock

from std_msgs.msg import String
from autoware_perception_msgs.msg import TrafficSignalArray, TrafficSignal, TrafficSignalElement
from traffic_light_signal_msgs.msg import TrafficLightSignalArray, TrafficLightSignal


# yolov5_tl202308 -> traffic_light_arbitor/arbitor
# [(color, shape, status), ...]
CLASS_CONVERT_TL202308 = {
    "green": # 0
    [(TrafficSignalElement.GREEN, TrafficSignalElement.CIRCLE, TrafficSignalElement.SOLID_ON)],
    "yellow": # 1
    [(TrafficSignalElement.AMBER, TrafficSignalElement.CIRCLE, TrafficSignalElement.SOLID_ON)],
    "red": # 2
    [(TrafficSignalElement.RED, TrafficSignalElement.CIRCLE, TrafficSignalElement.SOLID_ON)],
    "red_right": # 3
    [(TrafficSignalElement.RED, TrafficSignalElement.CIRCLE, TrafficSignalElement.SOLID_ON),
     (TrafficSignalElement.GREEN, TrafficSignalElement.RIGHT_ARROW, TrafficSignalElement.SOLID_ON)],
    "red_up": # 4
    [(TrafficSignalElement.RED, TrafficSignalElement.CIRCLE, TrafficSignalElement.SOLID_ON),
     (TrafficSignalElement.GREEN, TrafficSignalElement.UP_ARROW, TrafficSignalElement.SOLID_ON)],
    "red_left": # 5
    [(TrafficSignalElement.RED, TrafficSignalElement.CIRCLE, TrafficSignalElement.SOLID_ON),
     (TrafficSignalElement.GREEN, TrafficSignalElement.LEFT_ARROW, TrafficSignalElement.SOLID_ON)],
    "red_left_up": # 6
    [(TrafficSignalElement.RED, TrafficSignalElement.CIRCLE, TrafficSignalElement.SOLID_ON),
     (TrafficSignalElement.GREEN, TrafficSignalElement.LEFT_ARROW, TrafficSignalElement.SOLID_ON),
     (TrafficSignalElement.GREEN, TrafficSignalElement.UP_ARROW, TrafficSignalElement.SOLID_ON)],
    "unknown": # X
    [(TrafficSignalElement.UNKNOWN, TrafficSignalElement.UNKNOWN, TrafficSignalElement.UNKNOWN)],
}

SIGNAL_OUT_INTAERVAL_TIME = (1.0 / 30)

class TrafficSignalPub(Node):
    def __init__(self):
        super().__init__('traffic_signal_pub')
        print("start traffic_signal_pub")
        # Get parameters
        self.declare_parameter('traffic_light_id', 2180)
        self.traffic_light_id = self.get_parameter('traffic_light_id').get_parameter_value().integer_value
        self.declare_parameter('traffic_light_class', "red")
        self.traffic_light_class = self.get_parameter('traffic_light_class').get_parameter_value().string_value
        self.declare_parameter('interval_time', SIGNAL_OUT_INTAERVAL_TIME)
        self.interval_time = self.get_parameter('interval_time').get_parameter_value().double_value
        # Create Subscribers
        self.light_signals_sub = self.create_subscription(TrafficLightSignalArray, "/input/light_signals", self.light_signals_callback, 10)
        # Create Publishers
        self.detect_traffic_signals_pub = self.create_publisher(TrafficSignalArray, "/output/class/traffic_signals", 10)
        self.debug_msg_pub = self.create_publisher(String, '/output/debug/msg', 20)
        # Interval timer
        self.signal_out_timer = self.create_timer(self.interval_time, self.signal_out)
        # Initialize internal variables
        self.light_signals = None

    def light_signals_callback(self, msg):
        self.light_signals = msg

    def signal_out(self):
        if self.light_signals is None:
            return
        traffic_signal_id = None
        light_signal_id = TrafficLightSignal()
        light_signals_array = self.light_signals.array
        for no, light_signal_id in enumerate(light_signals_array):
            # print(no, light_signal_id)
            if light_signal_id.traffic_light_id == self.traffic_light_id:
                traffic_signal_id = light_signal_id.traffic_signal_id
                break
        if traffic_signal_id is not None:
            traffic_signals = TrafficSignalArray()
            traffic_signals.stamp = self.get_clock().now().to_msg()
            traffic_signal = TrafficSignal()
            traffic_signal.traffic_signal_id = traffic_signal_id
            if self.traffic_light_class in CLASS_CONVERT_TL202308:
                req_elements = CLASS_CONVERT_TL202308[self.traffic_light_class]
                for color, shape, status in req_elements:
                    element = TrafficSignalElement()
                    element.color = color
                    element.shape = shape
                    element.status = status
                    traffic_signal.elements.append(element)
                traffic_signals.signals.append(traffic_signal)
                self.detect_traffic_signals_pub.publish(traffic_signals)
                #print(traffic_signals)

def main(args=None):
    rclpy.init(args=args)
    main_node = TrafficSignalPub()
    try:
        rclpy.spin(main_node)
    except KeyboardInterrupt:
        print("Shutting down")

    main_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
