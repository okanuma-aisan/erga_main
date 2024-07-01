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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUReverse(Node):
    def __init__(self):
        super().__init__("imu_reversed")
        self.data = 0.0
        self.sub = self.create_subscription(Imu, "/sensing/imu/imu_data", self.on_imu_data, 1)
        self.pub = self.create_publisher(Imu, "/sensing/imu/imu_data/reversed", 1)

    def on_imu_data(self, msg):
        imu = Imu()
        imu = msg
        imu.angular_velocity.z = -msg.angular_velocity.z

        try:
            self.pub.publish(imu)
            print("Publish reversed imu msg!")
        except ValueError:
            print("Value error when publishing reversed imu msg!!")


def main(args=None):
    rclpy.init(args=args)

    reverse = IMUReverse()

    rclpy.spin(reverse)

    reverse.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
