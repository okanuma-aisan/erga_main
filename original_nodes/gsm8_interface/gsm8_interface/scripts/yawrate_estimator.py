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

import math

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node

from gsm8_interface_msgs.msg import M0Status
from gsm8_interface_msgs.msg import M3Status

WHEEL_BASE = 2.85  # [m]


class YawrateEstimator(Node):
    def __init__(self):
        super().__init__("eps_voltage_tester")
        self.sub_vehicle_status_m0 = self.create_subscription(
            M0Status, "/gsm8/can/status/m0_status", self.on_m0_status, 1
        )
        self.sub_vehicle_status_m3 = self.create_subscription(
            M3Status, "/gsm8/can/status/m3_status", self.on_m3_status, 1
        )
        self.pub_yawrate = self.create_publisher(TwistStamped, "kinematic_bicycle_model/twist", 10)

        self.vehicle_speed = 0.0
        self.steering_angle = 0.0

    def on_m0_status(self, msg):
        self.vehicle_speed = msg.spinv

    def on_m3_status(self, msg):
        self.steering_angle = msg.str

        yaw_rate = -(self.vehicle_speed * math.tan(self.steering_angle)) / WHEEL_BASE

        twist = TwistStamped()
        twist.header.stamp = msg.header.stamp
        twist.twist.linear.x = self.vehicle_speed
        twist.twist.angular.z = yaw_rate

        try:
            self.pub_yawrate.publish(twist)
            print("Publish /kinematic_bicycle_model/twist msg!")
        except ValueError:
            print("Value error when publishing /kinematic_bicycle_model/twist !!")
            pass


def main(args=None):
    rclpy.init(args=args)

    estimator = YawrateEstimator()

    rclpy.spin(estimator)

    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
