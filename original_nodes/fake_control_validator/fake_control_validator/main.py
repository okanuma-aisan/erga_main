import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, String
from control_validator.msg import ControlValidatorStatus

class MainNode(Node):
    def __init__(self):
        super().__init__('fake_control_validator')
        self.publisher_ = self.create_publisher(ControlValidatorStatus, '/control/control_validator/validation_status', 10)
        timer_period = 1 / 300  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        print("hello")
        msg = ControlValidatorStatus()
        msg.is_valid_max_distance_deviation = True
        msg.max_distance_deviation = 0.0
        msg.invalid_count = 0
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

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
