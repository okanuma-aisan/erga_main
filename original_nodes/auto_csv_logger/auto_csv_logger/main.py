import rclpy
from rclpy.node import Node
from ros2topic.api import get_message, get_msg_class
from rosidl_runtime_py import message_to_ordereddict
from collections import OrderedDict
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from datetime import datetime
from pathlib import Path
from std_msgs.msg import Bool
import time
import inspect

builtins = ['bool', 'boolean', 'byte', 'char', 'double', 'float', 'float32', 'float64', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'string', 'wstring']
def to_builtins(builtin_str: str) -> any:
    match builtin_str:
        case 'bool':
            return bool
        case 'boolean': # ???? documentation doesnt mention this
            return bool
        case 'byte':
            return bytes
        case 'char':
            return str
        case 'double': # ???? documentation doesnt mention this
            return float
        case 'float':
            return float
        case 'float32':
            return float
        case 'float64':
            return float
        case 'int8':
            return int
        case 'uint8':
            return int
        case 'int16':
            return int
        case 'uint16':
            return int
        case 'int32':
            return int
        case 'uint32':
            return int
        case 'int64':
            return int
        case 'uint64':
            return int
        case 'string':
            return str
        case 'wstring':
            return str
    # process array
    if builtin_str[-1] == ']':
        type_str, length = builtin_str[:-1].rsplit('[', 1)
        builtin_type = to_builtins(type_str)
        return [builtin_type] * int(length)

def recurse_message_metaclass(msg_class: any):
    fields_and_types = OrderedDict()
    fields_types = msg_class.get_fields_and_field_types()
    for field_name in fields_types.keys():
        field_type = fields_types[field_name]
        if (field_type in builtins) or (field_type[-1] == "]"):
            fields_and_types[field_name] = to_builtins(field_type)
            continue
        fields_and_types[field_name] = recurse_message_metaclass(get_message(field_type))
    return fields_and_types

def message_or_metaclass_to_ordereddict(msg: any) -> OrderedDict:
    if type(msg).__name__[:10] == 'Metaclass_':
        return recurse_message_metaclass(msg)
    return message_to_ordereddict(msg)

def ordered_dict_to_field_value_pair(od, topic_path="", toplevel=True):
    keys = []
    for key, value in od.items():
        prefix = ""
        if toplevel:
            prefix = f"{topic_path}."
        if isinstance(value, OrderedDict):
            keys += [(f"{prefix}{key}.{x}", y) for x, y in ordered_dict_to_field_value_pair(value, topic_path, toplevel=False)]
            continue
        keys.append((f"{prefix}{key}", value))
    return keys

def ordered_dict_to_flat_list(ordered_dict, isheader=False) -> list['str']:
        flattened = []
        for topic_name, value_list in ordered_dict.items():
            for field, value in value_list:
                # for headers we only want the field
                if isheader:
                    flattened.append(field)
                    continue
                if type(value) is type:
                    flattened.append('')
                    continue
                if type(value) is list:
                    flattened.append(str(value).replace(', ', ';'))
                    continue
                flattened.append(value)
        flattened = [str(x) for x in flattened]
        return flattened

class MainNode(Node):

    def __init__(self):
        super().__init__('auto_csv_logger')
        self.logger_ = self.get_logger()
        self.ros_clock_ = self.get_clock()
        self.log_path_ = self.declare_parameter('log_path', '/home/sit/csv_logger', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Path to store recorded log files."))
        self.filename_format_ = self.declare_parameter('filename_format', "%Y-%m-%d_%H-%M-%S.csv", ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Format string for log file."))
        self.log_topics_ = self.declare_parameter('log_topics', '/chatter', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="Topics to log, separated by ;"))

        # topics to-be-logged enumeration and subscription logic
        self.log_topics_list_ = self.log_topics_.get_parameter_value().string_value.split(';')
        if len(self.log_topics_list_) < 1:
            self.logger_.error("Provided list of topics is empty.")
            self.destroy_node()

        self.class_log_topics_mapper = {}
        self.log_topics_class_mapper = {}
        self.subscribers_list_ = [] # prevent garbage collection
	
        self.logger_.info("Waiting for all topics to be available ...")
        while True:
            available = self.all_topics_available()
            if available:
                break
            time.sleep(1)

        self.accumulated_topics_ = OrderedDict()
        self.logger_.info("All topics found. Creating subscribers ...")
        for topic_idx, topic_path in enumerate(self.log_topics_list_):
            message_class = get_msg_class(self, topic_path, include_hidden_topics=True, blocking=True)
            self.accumulated_topics_[topic_path] = ordered_dict_to_field_value_pair(message_or_metaclass_to_ordereddict(message_class), topic_path=topic_path)
            subscriber = self.create_subscription(message_class, topic_path, lambda msg: self.subscription_callback(msg, topic_path), 10)
            self.logger_.info(f"Subscribing to {topic_path} with type {message_class}.")
            self.subscribers_list_.append(subscriber) # prevent garbage collection
        # logger start / stop logic
        self.subscriber_log_trigger_ = self.create_subscription(Bool, '~/log_trigger', self.log_trigger_callback_, 10)
        self.header_written_ = False
        self.previous_logging_state_ = None
        self.logging_ = False
        self.log_file_ = None
        self.log_fp_ = None
        self.logger_.info("Initialization completed. Waiting for logging trigger ...")

    def all_topics_available(self) -> None:
        for topic_idx, topic_path in enumerate(self.log_topics_list_):
            message_class = get_msg_class(self, topic_path, include_hidden_topics=True)
            if message_class is None:
                self.logger_.warn(f"{topic_path} isn't available. Logging cannot begin.")
                return False
        return True

    def generate_log_path(self) -> Path:
        destination_path = Path(self.log_path_.get_parameter_value().string_value)
        if not destination_path.is_dir():
            destination_path.mkdir()
        return destination_path.joinpath(str(datetime.now().strftime(self.filename_format_.get_parameter_value().string_value)))

    def log_trigger_callback_(self, msg: Bool):
        if msg.data == self.previous_logging_state_:
            return
        self.previous_logging_state_ = msg.data
        if msg.data == False: # reset variables and stop logging
            if self.logging_ is False:
                return
            self.logger_.info(f"Stopped logging. Destination file is at '{self.log_file_}'.")
            self.logging_ = False
            self.header_written_ = False
            self.accumulated_topics_ = {}
            if self.log_fp_ is not None:
                self.log_fp_.close()
            return
        
        self.log_file_ = self.generate_log_path()
        self.log_fp_ = self.log_file_.open('w')
        self.logger_.info(f"Started logging. Destination file is '{self.log_file_}'.")
        self.logging_ = True

    def get_timestamp(self) -> float:
        secs, nanosecs = self.ros_clock_.now().seconds_nanoseconds()
        return str(secs + nanosecs * 0.000000001)
    
    def check_subscriptions(self) -> bool:
        if len(self.accumulated_topics_.keys()) != len(self.log_topics_list_):
            return False
        return True
            
    def subscription_callback(self, msg, topic_path):
        self.accumulated_topics_[topic_path] = ordered_dict_to_field_value_pair(message_or_metaclass_to_ordereddict(msg), topic_path=topic_path)
        if self.logging_ is False:
            return
        
        if not self.header_written_:
            header = ordered_dict_to_flat_list(self.accumulated_topics_, isheader=True)
            self.log_fp_.write(f"ros_timestamp,{','.join(header)}\n")
            self.header_written_ = True

        keys = ordered_dict_to_flat_list(self.accumulated_topics_, isheader=False)
        self.log_fp_.write(f"{self.get_timestamp()},{','.join(keys)}\n")
        self.log_fp_.flush()

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
