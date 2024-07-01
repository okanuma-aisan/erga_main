import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, String

class MainNode(Node):

    def __init__(self):
        super().__init__('autoware_fms_offline_gui')
        self.publisher_ = self.create_publisher(String, '/template_pub', 10)
        self.subscriber_ = self.create_subscription(String, '/template_sub', self.subscription_callback, 10)
        timer_period = 1 / 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello"

        msg_header = Header()
        msg_header.stamp = self.get_clock().now().to_msg()
        msg_header.frame_id = ""
        
        self.publisher_.publish(msg)

    def subscription_callback(self, msg):
        print(msg)

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
