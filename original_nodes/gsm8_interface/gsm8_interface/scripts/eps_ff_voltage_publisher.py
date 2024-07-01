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

from collections import deque

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand
import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import Float32MultiArrayStamped

DELAY = 0.12  # [sec]
DT = 0.02  # [sec]


class EPSFFVoltagePublisher(Node):
    def __init__(self):
        super().__init__("eps_ff_voltage_publisher")
        self.current_steer_angle = 0.0
        self.target_steer_angle = 0.0
        self.target_steer_angle_velocity = 0.0

        self.sub = self.create_subscription(
            Float32MultiArrayStamped, "/vehicle/tester/eps_ff_voltage", self.on_eps_ff_voltage, 1
        )
        self.pub_control_cmd = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd", 1
        )
        self.pub_gear_cmd = self.create_publisher(GearCommand, "/control/command/gear_cmd", 1)
        self.timer = self.create_timer(DT, self.on_timer)
        self.queue = deque(maxlen=int(DELAY / DT))

    def on_eps_ff_voltage(self, msg):
        self.target_steer_angle = msg.data[0]
        self.target_steer_angle_velocity = msg.data[1]
        print("----------------------------------------------------------------------")
        print(f"update! target_steer_angle: {self.target_steer_angle}")
        print(f"update! target_steer_angle_velocity: {self.target_steer_angle_velocity}")
        print("----------------------------------------------------------------------")

    def on_timer(self):
        if self.target_steer_angle_velocity > 0:
            self.current_steer_angle += self.target_steer_angle_velocity * DT
            if self.current_steer_angle > self.target_steer_angle:
                self.current_steer_angle = self.target_steer_angle
                self.target_steer_angle_velocity = 0.0
        elif self.target_steer_angle_velocity < 0:
            self.current_steer_angle += self.target_steer_angle_velocity * DT
            if self.current_steer_angle < self.target_steer_angle:
                self.current_steer_angle = self.target_steer_angle
                self.target_steer_angle_velocity = 0.0

        if len(self.queue) == self.queue.maxlen:
            delayed_steer_angle = self.queue.popleft()
            self.queue.append(self.current_steer_angle)
        else:
            delayed_steer_angle = 0.0
            self.queue.append(self.current_steer_angle)

        msg_control_cmd = AckermannControlCommand()
        msg_control_cmd.stamp = self.get_clock().now().to_msg()
        msg_control_cmd.lateral.stamp = self.get_clock().now().to_msg()
        msg_control_cmd.lateral.steering_tire_angle = delayed_steer_angle
        msg_control_cmd.lateral.steering_tire_rotation_rate = self.target_steer_angle_velocity
        msg_control_cmd.longitudinal.stamp = self.get_clock().now().to_msg()
        msg_control_cmd.longitudinal.speed = 0.0
        msg_control_cmd.longitudinal.acceleration = 0.0
        msg_control_cmd.longitudinal.jerk = 0.0
        self.pub_control_cmd.publish(msg_control_cmd)

        msg_gear_cmd = GearCommand()
        msg_gear_cmd.stamp = self.get_clock().now().to_msg()
        msg_gear_cmd.command = GearCommand.DRIVE
        self.pub_gear_cmd.publish(msg_gear_cmd)

        print(
            f"publish VehicleCommand with eps steering angle: {self.current_steer_angle}"
            + f" vel: {self.target_steer_angle_velocity}"
        )


def main(args=None):
    rclpy.init(args=args)

    eps_ff_voltage_publisher = EPSFFVoltagePublisher()

    rclpy.spin(eps_ff_voltage_publisher)

    eps_ff_voltage_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
