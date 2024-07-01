import rclpy
from rclpy.node import Node

import socket
import struct

from std_msgs.msg import Header, String
from autoware_perception_msgs.msg import TrafficSignalArray, TrafficSignal, TrafficSignalElement
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import xml.etree.ElementTree as ET
import pathlib

def get_all_traffic_light_ids(logger, osm_path=pathlib.Path) -> list[int]:
    tree = ET.parse(osm_path)

    final_traffic_ids = []
    root_node = tree.getroot()
    for entry in root_node.iter():
        entry_type = None
        entry_subtype = None
        for tags in entry.iter():
            match tags.get('k'):
                case 'type':
                    entry_type = tags.get('v')
                case 'subtype':
                    entry_subtype = tags.get('v')
        if entry_type != "regulatory_element":
            continue
        if entry_subtype != "traffic_light":
            continue
        if entry.get('id') is None:
            logger.warn("Map may be invalid, traffic light element has no ID!")
            continue
        final_traffic_ids.append(int(entry.get('id')))
    return final_traffic_ids

class UDPTrafficSignalStatus:
    def __init__(self, class_id: int, confidence: float, xmin: float, xmax: float, ymin: float, ymax: float, class_name: str):
        # yolo data itself
        self.class_id = class_id
        self.confidence = confidence
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.class_name = class_name

        # autoware TrafficSignalElement
        self.autoware_element = self.create_autoware_element()

    def create_autoware_element(self) -> TrafficSignalElement:
        # create autoware data
        color = TrafficSignalElement.RED
        shape = TrafficSignalElement.CIRCLE

        match self.class_id:
            # circle (no directional info)
            case 0:
                color = TrafficSignalElement.GREEN
            case 1:
                color = TrafficSignalElement.AMBER
            case 2:
                color = TrafficSignalElement.RED

            # with directional info
            case 3:
                color = TrafficSignalElement.RED
                shape = TrafficSignalElement.RIGHT_ARROW
            case 4:
                color = TrafficSignalElement.RED
                shape = TrafficSignalElement.UP_ARROW
            case 5:
                color = TrafficSignalElement.RED
                shape = TrafficSignalElement.LEFT_ARROW
            case 6:
                color = TrafficSignalElement.RED
                shape = TrafficSignalElement.UP_LEFT_ARROW

        out_element = TrafficSignalElement()
        out_element.color = color
        out_element.shape = shape
        out_element.status = TrafficSignalElement.SOLID_ON
        out_element.confidence = self.confidence

        return out_element

class MainNode(Node):

    def __init__(self):
        super().__init__('nshinjuku_traffic_signal')
        self.last_received_traffic_signal_data_ = None

        self.autoware_traffic_light_publisher_ = self.create_publisher(TrafficSignalArray, '~/pub/external_traffic_signals', 10)
        self.map_path_ = self.declare_parameter('lanelet2_map_file', 'lanelet2_map.osm', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Path to Lanelet2 map file."))
        self.logger_ = self.get_logger()

        self.traffic_light_ids_ = []
        map_path = pathlib.Path(self.map_path_.get_parameter_value().string_value)
        self.logger_.info(f"lanelet2_map_file={self.map_path_.get_parameter_value().string_value}")
        self.logger_.info(f"Retrieving traffic light ids from {map_path} ...")
        try:
            self.traffic_light_ids_ = get_all_traffic_light_ids(self.logger_, map_path)
        except Exception as e:
            self.logger_.fatal(f"Failed to retrieve traffic light ids. Reason: {e}")
            exit(1)
        if len(self.traffic_light_ids_) < 1:
            self.logger_.fatal("Failed to retrieve traffic light ids.")
            exit(1)

        self.socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.listen_address_ = self.declare_parameter('listen_address', '0.0.0.0', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Address to listen on"))
        self.listen_port_ = self.declare_parameter('listen_port', 9385, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description="Port to listen on"))

        listen_address = self.listen_address_.get_parameter_value().string_value
        listen_port = self.listen_port_.get_parameter_value().integer_value

        self.socket_.bind(((listen_address, listen_port)))

        self.override_ = False

        self.start_receiving_()
    
    def recv_and_decode_(self) -> list[UDPTrafficSignalStatus]:
        output = []
        recv_data = self.socket_.recv(4096)
        if recv_data[0] == 0x6a: # start override
            self.override_ = True
        if recv_data[0] == 0x6b: # stop override
            self.override_ = False
            return
        if self.override_ and recv_data[0] == 0x6c: # ignore detections if overriding
            #self.logger_.warn("Signal being overridden ...")
            return
        if not self.override_ and recv_data[0] != 0x6c: # unknown data
            self.logger_.warn("Disregarding unknown message ...")
            return

        traffic_signal_data_count = recv_data[1]
        for i in range(0, traffic_signal_data_count):
            offset = i*56
            traffic_data = recv_data[2+offset:2+offset+56]
            class_id, confidence, xmin, xmax, ymin, ymax, class_name = struct.unpack("Bfffff32s", traffic_data)
            class_name.decode('ascii')
            obj = UDPTrafficSignalStatus(class_id, confidence, xmin, xmax, ymin, ymax, class_name)
            output.append(obj)
        return output
    
    def start_receiving_(self):
        self.logger_.info("Started message retrieval.")
        while True:
            self.receive_and_publish_()

    def receive_and_publish_(self):
        signal_data_recv_ = self.recv_and_decode_()
        if signal_data_recv_ is None:
            return
        
        signal_element_list = []
        for traffic_signal in signal_data_recv_:
            autoware_element = traffic_signal.autoware_element
            signal_element_list.append(autoware_element)
       	if len(signal_element_list) < 1:
            autoware_final_signal_array = TrafficSignalArray()
            autoware_final_signal_array.stamp = self.get_clock().now().to_msg()
            autoware_final_signal_array.signals = [] # this should be for multiple lights, but both traffic lights are the same for this
        
            self.autoware_traffic_light_publisher_.publish(autoware_final_signal_array) 
            return

        traffic_signal_msgs = []
        for pub_id in self.traffic_light_ids_:
            autoware_signal = TrafficSignal()
            autoware_signal.traffic_signal_id = pub_id
            autoware_signal.elements = signal_element_list
            traffic_signal_msgs.append(autoware_signal)

        autoware_final_signal_array = TrafficSignalArray()
        autoware_final_signal_array.stamp = self.get_clock().now().to_msg()
        autoware_final_signal_array.signals = traffic_signal_msgs # this should be for multiple lights, but both traffic lights are the same for this
        
        self.autoware_traffic_light_publisher_.publish(autoware_final_signal_array) 

def main(args=None):
    rclpy.init(args=args)

    main_node = MainNode()

    rclpy.spin(main_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    main_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
