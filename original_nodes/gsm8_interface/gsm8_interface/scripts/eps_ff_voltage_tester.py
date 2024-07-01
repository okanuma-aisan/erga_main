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
from tier4_debug_msgs.msg import Float32MultiArrayStamped

MAX_STEER_ANGLE = 0.5  # [rad]
MIN_STEER_ANGLE = -0.5  # [rad]
MAX_STEER_ANGLE_VELOCITY = 0.2  # [rad/s]
MIN_STEER_ANGLE_VELOCITY = -0.2  # [rad/s]


class EPSFFVoltageTester(Node):
    def __init__(self):
        super().__init__("eps_ff_voltage_tester")
        self.pub = self.create_publisher(
            Float32MultiArrayStamped, "/vehicle/tester/eps_ff_voltage", 1
        )

    def run(self):
        while rclpy.ok():
            target_steer_angle = float(
                input(f"target steer angle [{MIN_STEER_ANGLE} ~ {MAX_STEER_ANGLE} rad] > ")
            )
            target_steer_angle_velocity = float(
                input(
                    "target steer angle velocity "
                    + f"[{MIN_STEER_ANGLE_VELOCITY} ~ {MAX_STEER_ANGLE_VELOCITY}] rad/s] > "
                )
            )

            if target_steer_angle > MAX_STEER_ANGLE:
                print(
                    "input value is larger than max eps steer angle velocity!"
                    + f"input: {target_steer_angle} max: {MAX_STEER_ANGLE}"
                )
                target_steer_angle = MAX_STEER_ANGLE
            elif target_steer_angle < MIN_STEER_ANGLE:
                print(
                    "input value is smaller than min eps steer angle velocity!"
                    + f"input: {target_steer_angle} min: {MIN_STEER_ANGLE}"
                )
                target_steer_angle = MIN_STEER_ANGLE

            if target_steer_angle_velocity > MAX_STEER_ANGLE_VELOCITY:
                print(
                    "input value is larger than max eps steer angle velocity!"
                    + f"input: {target_steer_angle_velocity}, max: {MAX_STEER_ANGLE_VELOCITY}"
                )
                target_steer_angle_velocity = MAX_STEER_ANGLE_VELOCITY
            elif target_steer_angle_velocity < MIN_STEER_ANGLE_VELOCITY:
                print(
                    "input value is smaller than min eps steer angle velocity!"
                    + f"input: {target_steer_angle_velocity} min: {MIN_STEER_ANGLE_VELOCITY}"
                )
                target_steer_angle_velocity = MIN_STEER_ANGLE_VELOCITY

            print(
                f"target_steer_angle: {target_steer_angle}, "
                + f"target_steer_angle_velocity: {target_steer_angle_velocity}"
            )

            content = [target_steer_angle, target_steer_angle_velocity]
            msg = Float32MultiArrayStamped(stamp=self.get_clock().now().to_msg(), data=content)
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    tester = EPSFFVoltageTester()
    tester.run()

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
