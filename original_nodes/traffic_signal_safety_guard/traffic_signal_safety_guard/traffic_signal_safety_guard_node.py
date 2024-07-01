import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from autoware_perception_msgs.msg import TrafficSignalArray, TrafficSignal, TrafficSignalElement
from sensor_msgs.msg import CameraInfo

from copy import deepcopy

CAMERA_NUM = 2

DEFAULT_CAMERA_INFO_TIMEOUT = 1.0  # camera_infoの最大sub間隔時間
DEFAULT_INTERVAL_TIME = (1 / 15)  # 赤信号publish間隔

class TrafficSignalSafetyGuard(Node):
    def __init__(self):
        super().__init__('traffic_signal_safety_guard')
        print("start traffic_signal_safety_guard")
        camera_info_profile = qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Get Parameters
        self.declare_parameter('camera_info_timeout', DEFAULT_CAMERA_INFO_TIMEOUT)
        self.camera_info_timeout = self.get_parameter('camera_info_timeout').get_parameter_value().double_value
        self.declare_parameter('interval_time', DEFAULT_INTERVAL_TIME)
        self.interval_time = self.get_parameter('interval_time').get_parameter_value().double_value
        # Create Subscribers
        self.traffic_signals_sub = self.create_subscription(TrafficSignalArray, "/input/traffic_signals", self.traffic_signals_callback, 10)
        self.camera_info_sub1 = self.create_subscription(CameraInfo, "/input/camera_info1", self.camera_info1_callback, qos_profile=camera_info_profile)
        self.camera_info_sub2 = self.create_subscription(CameraInfo, "/input/camera_info2", self.camera_info2_callback, qos_profile=camera_info_profile)
        # Create Publishers
        self.traffic_signals_pub = self.create_publisher(TrafficSignalArray, "/output/traffic_signals", 10)
        # Interval timer for monitoring camera_info
        self.camera_info_monitoring_timer = self.create_timer(self.camera_info_timeout, self.camera_info_monitoring)
        self.traffic_signals_publish_timer = self.create_timer(self.interval_time, self.traffic_signals_publish)
        # Initialize internal variables
        self.camera_info_count = [0] * CAMERA_NUM
        self.camera_info_count_prev = [0] * CAMERA_NUM
        self.camera_info = [None] * CAMERA_NUM
        self.traffic_signals = None
        self.traffic_signals_out = None
        self.req_red_flag1 = False
        self.req_red_flag2 = False

    def camera_info1_callback(self, msg: CameraInfo):
        self.update_camera_info(0, msg)

    def camera_info2_callback(self, msg: CameraInfo):
        self.update_camera_info(1, msg)

    def update_camera_info(self, camera_no, msg):
        self.camera_info[camera_no] = msg
        self.camera_info_count[camera_no] += 1
        #print(f"camera_info{camera_no+1}: {self.camera_info_count[camera_no]}")

    def traffic_signals_callback(self, msg: TrafficSignalArray):
        self.traffic_signals = msg
        new_traffic_signals = deepcopy(self.traffic_signals)
        signals = new_traffic_signals.signals
        out_flag = False
        for signal in signals:
            elements = signal.elements
            for element in elements:
                # 信号情報にUNKNOWNが含まれる場合は、強制的に赤信号にする
                if ((element.color == TrafficSignalElement.UNKNOWN) or
                    (element.shape == TrafficSignalElement.UNKNOWN) or 
                    (element.status == TrafficSignalElement.UNKNOWN)):
                    #print(f"unknown traffic signals -> RED")
                    out_flag = True
                    break
        # 結果格納
        self.req_red_flag1 = out_flag


    def camera_info_monitoring(self):
        # camera_infoが指定時間の間にsubscribeされなかった場合は、強制的に赤信号にする
        out_flag = False
        #print(f"camera_info counter ({self.camera_info_count_prev},{self.camera_info_count})")
        if ((self.camera_info_count[0] == self.camera_info_count_prev[0]) and
            (self.camera_info_count[1] == self.camera_info_count_prev[1])):
            #print(self.traffic_signals)
            if (self.traffic_signals is not None) and len(self.traffic_signals.signals):
                signals = self.traffic_signals.signals
                for signal in signals:
                    #print(f"no camera_info ({self.camera_info_count_prev},{self.camera_info_count}) : traffic_signal_id={signal.traffic_signal_id} -> RED")
                    new_element = TrafficSignalElement()
                    new_element.color = TrafficSignalElement.RED
                    new_element.shape = TrafficSignalElement.CIRCLE
                    new_element.status = TrafficSignalElement.SOLID_ON
                    new_element.confidence = 1.0
                    signal.elements = []
                    signal.elements.append(new_element)
                    out_flag = True
        # 結果格納
        self.req_red_flag2 = out_flag
        # カウンタの更新
        for camera_no in range(CAMERA_NUM):
            self.camera_info_count_prev[camera_no] = self.camera_info_count[camera_no]

    def traffic_signals_publish(self):
        if self.traffic_signals is not None:
            if (not self.req_red_flag1) and (not self.req_red_flag2):
                # 元のtraffic_signals
                self.traffic_signals_pub.publish(self.traffic_signals)
            else:
                # 赤信号のtraffic_signals
                new_traffic_signals = deepcopy(self.traffic_signals)
                signals = new_traffic_signals.signals
                for signal in signals:
                    traffic_signal_id = signal.traffic_signal_id
                    print(f"traffic_signal_id={traffic_signal_id} : RED")
                    new_element = TrafficSignalElement()
                    new_element.color = TrafficSignalElement.RED
                    new_element.shape = TrafficSignalElement.CIRCLE
                    new_element.status = TrafficSignalElement.SOLID_ON
                    new_element.confidence = 1.0
                    signal.elements = []
                    signal.elements.append(new_element)
                self.traffic_signals_pub.publish(new_traffic_signals)

def main(args=None):
    rclpy.init(args=args)
    main_node = TrafficSignalSafetyGuard()
    try:
        rclpy.spin(main_node)
    except KeyboardInterrupt:
        print("Shutting down")

    main_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
