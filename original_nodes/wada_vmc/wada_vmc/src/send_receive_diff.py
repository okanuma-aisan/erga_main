#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from wada_vmc_msgs.msg import Can502_20210103
from wada_vmc_msgs.msg import Can100_20210103

class SendReceiveDiff(Node):
	def __init__(self):
		super().__init__('send_receive_diff')

		self.sub_can100 = self.create_subscription(Can100_20210103, '/vehicle/can100', self.can100callback, 1)
		self.sub_can502 = self.create_subscription(Can502_20210103, '/vehicle/can502', self.can502callback, 1)

		self.pub_data = self.create_publisher(Int16MultiArray, 'steer_data', 1)

		self.timer = self.create_timer(1.0/100.0, self.timer_callback)

		self.send_steer = 0
		self.recv_steer = 0

	def can100callback(self, msg):
		self.send_steer = msg.steer_command

	def can502callback(self, msg):
		self.recv_steer = msg.steering_angle

	def timer_callback(self):
		msg_steer_data = Int16MultiArray()
		msg_steer_data.data = [self.send_steer, self.recv_steer]
		self.pub_data.publish(msg_steer_data)

def main(args=None):
	rclpy.init(args=args)

	node = SendReceiveDiff()
	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()