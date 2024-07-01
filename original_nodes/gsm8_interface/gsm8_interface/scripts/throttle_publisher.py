#!/usr/bin/env python3

# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from autoware_auto_vehicle_msgs.msg import GearCommand
import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import Float32Stamped
from tier4_vehicle_msgs.msg import ActuationCommandStamped


class ThrottlePublisher(Node):
    def __init__(self):
        super().__init__("throttle_publisher")
        self.target_throttle = 0.0

        self.sub = self.create_subscription(
            Float32Stamped, "/vehicle/tester/throttle", self.on_throttle, 1
        )
        self.pub_actuation_cmd = self.create_publisher(
            ActuationCommandStamped, "/control/command/actuation_cmd", 1
        )
        self.pub_gear_cmd = self.create_publisher(GearCommand, "/control/command/gear_cmd", 1)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.on_timer)

    def on_throttle(self, msg):
        self.target_throttle = msg.data
        print(f"Set target throttle: {self.target_throttle}")

    def on_timer(self):
        msg_actuation_cmd = ActuationCommandStamped()
        msg_actuation_cmd.actuation.brake_cmd = 0.0
        msg_actuation_cmd.actuation.steer_cmd = 0.0
        msg_actuation_cmd.header.stamp = self.get_clock().now().to_msg()
        msg_actuation_cmd.header.frame_id = "base_link"
        msg_actuation_cmd.actuation.accel_cmd = self.target_throttle
        self.pub_actuation_cmd.publish(msg_actuation_cmd)

        msg_gear_cmd = GearCommand()
        msg_gear_cmd.stamp = self.get_clock().now().to_msg()
        msg_gear_cmd.command = GearCommand.DRIVE
        self.pub_gear_cmd.publish(msg_gear_cmd)
        print(f"publish ActuationCommand with accel value: {self.target_throttle}")


def main(args=None):
    rclpy.init(args=args)

    throttle_publisher = ThrottlePublisher()

    rclpy.spin(throttle_publisher)

    throttle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
