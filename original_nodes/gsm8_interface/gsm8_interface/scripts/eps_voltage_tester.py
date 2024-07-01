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
from tier4_debug_msgs.msg import Float32Stamped

MAX_VOLTAGE = 12.0  # [V]
MIN_VOLTAGE = -12.0  # [V]


class EPSVoltageTester(Node):
    def __init__(self):
        super().__init__("eps_voltage_tester")
        self.pub = self.create_publisher(Float32Stamped, "/vehicle/tester/eps_voltage", 1)

    def run(self):
        while rclpy.ok():
            value = float(input(f"target eps voltage [{MIN_VOLTAGE} ~ {MAX_VOLTAGE} V] > "))
            if value > MAX_VOLTAGE:
                print(
                    "input value is larger than max eps voltage! "
                    + f"input: {value}, max: {MAX_VOLTAGE}"
                )
                value = MAX_VOLTAGE
            elif value < MIN_VOLTAGE:
                print(
                    "input value is smaller than min eps voltage! "
                    + f"input: {value}, min: {MIN_VOLTAGE}"
                )
                value = MIN_VOLTAGE

            msg = Float32Stamped(stamp=self.get_clock().now().to_msg(), data=value)
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    tester = EPSVoltageTester()
    tester.run()

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
