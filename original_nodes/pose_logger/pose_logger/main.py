import rclpy
import datetime
import math
import time
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

def quaternion_to_rpy(quat):
    q_x, q_y, q_z, q_w = quat
    q_x = float(q_x)
    q_y = float(q_y)
    q_z = float(q_z)
    q_w = float(q_w)
    roll = math.atan2(2.0*(q_y*q_z + q_w*q_x), q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z)
    pitch = math.asin(-2.0*(q_x*q_z - q_w*q_y))
    yaw = math.atan2(2.0*(q_x*q_y + q_w*q_z), q_w*q_w + q_x*q_x - q_y*q_y - q_z*q_z)
    return (roll, pitch, yaw)

def rpy_from_msg(msg):
    o_x = f"{msg.pose.orientation.x}"
    o_y = f"{msg.pose.orientation.y}"
    o_z = f"{msg.pose.orientation.z}"
    o_w = f"{msg.pose.orientation.w}"
    quat = (o_x, o_y, o_z, o_w)
    roll, pitch, yaw = quaternion_to_rpy(quat)
    return (roll, pitch, yaw)

class MainNode(Node):

    def __init__(self):
        super().__init__('pose_logger')
        #self.publisher_ = self.create_publisher(String, '/template_pub', 10)
        self.ekf_subscriber_ = self.create_subscription(PoseStamped, '/localization/pose_twist_fusion_filter/pose', self.ekf_subscription_callback, 10)
        self.gnss_subscriber_ = self.create_subscription(PoseStamped, '/sensing/gnss/pose', self.gnss_subscription_callback, 10)
        self.ndt_subscriber_ = self.create_subscription(PoseStamped, '/localization/pose_estimator/pose', self.ndt_subscription_callback, 10)
        self.log_file_name_ = f"/home/sit/pose_logger/{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        self.get_logger().info(f"Writing to log file: {self.log_file_name_}")
        self.log_file_ = open(self.log_file_name_, 'w')
        self.keys = [
            "timestamp",
            "ekf_x",
            "ekf_y",
            "ekf_z",
            "ekf_o_x",
            "ekf_o_y",
            "ekf_o_z",
            "ekf_o_w",
            "ekf_roll",
            "ekf_pitch",
            "ekf_yaw",

            "gnss_x",
            "gnss_y",
            "gnss_z",
            "gnss_o_x",
            "gnss_o_y",
            "gnss_o_z",
            "gnss_o_w",
            "gnss_roll",
            "gnss_pitch",
            "gnss_yaw",

            "ndt_x",
            "ndt_y",
            "ndt_z",
            "ndt_o_x",
            "ndt_o_y",
            "ndt_o_z",
            "ndt_o_w",
            "ndt_roll",
            "ndt_pitch",
            "ndt_yaw"
        ]
        self.log_file_.write(f"{','.join(self.keys)}\n")
        #self.log_file_.flush()
        #timer_period = 1 / 10  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ekf_msg = None
        self.gnss_msg = None
        self.ndt_msg = None
    
    def ekf_subscription_callback(self, msg):
        self.ekf_msg = msg
        self.accumulator_callback()

    def gnss_subscription_callback(self, msg):
        self.gnss_msg = msg
        self.accumulator_callback()

    def ndt_subscription_callback(self, msg):
        self.ndt_msg = msg
        self.accumulator_callback()

    def accumulator_callback(self):
        if self.ekf_msg is None:
            return
        if self.gnss_msg is None:
            return
        if self.ndt_msg is None:
            return

        ekf_roll, ekf_pitch, ekf_yaw = rpy_from_msg(self.ekf_msg)
        gnss_roll, gnss_pitch, gnss_yaw = rpy_from_msg(self.gnss_msg)
        ndt_roll, ndt_pitch, ndt_yaw = rpy_from_msg(self.ndt_msg)

        values = [
            f"{time.time()}",
            f"{self.ekf_msg.pose.position.x}",
            f"{self.ekf_msg.pose.position.y}",
            f"{self.ekf_msg.pose.position.z}",
            f"{self.ekf_msg.pose.orientation.x}",
            f"{self.ekf_msg.pose.orientation.y}",
            f"{self.ekf_msg.pose.orientation.z}",
            f"{self.ekf_msg.pose.orientation.w}",
            f"{ekf_roll}",
            f"{ekf_pitch}",
            f"{ekf_yaw}",

            f"{self.gnss_msg.pose.position.x}",
            f"{self.gnss_msg.pose.position.y}",
            f"{self.gnss_msg.pose.position.z}",
            f"{self.gnss_msg.pose.orientation.x}",
            f"{self.gnss_msg.pose.orientation.y}",
            f"{self.gnss_msg.pose.orientation.z}",
            f"{self.gnss_msg.pose.orientation.w}",
            f"{gnss_roll}",
            f"{gnss_pitch}",
            f"{gnss_yaw}",

            f"{self.ndt_msg.pose.position.x}",
            f"{self.ndt_msg.pose.position.y}",
            f"{self.ndt_msg.pose.position.z}",
            f"{self.ndt_msg.pose.orientation.x}",
            f"{self.ndt_msg.pose.orientation.y}",
            f"{self.ndt_msg.pose.orientation.z}",
            f"{self.ndt_msg.pose.orientation.w}",
            f"{ndt_roll}",
            f"{ndt_pitch}",
            f"{ndt_yaw}",
        ]
        self.log_file_.write(f"{','.join(values)}\n")
        self.log_file_.flush()

        self.ekf_msg = None
        self.gnss_msg = None
        self.ndt_msg = None
        
def main(args=None):
    rclpy.init(args=args)

    main_node = MainNode()

    rclpy.spin(main_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    main_node.destroy_node()
    main_node.log_file_.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
