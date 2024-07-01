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


from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import Float32Stamped


class AccelPublisher(Node):
    def __init__(self):
        super().__init__("acceleration_publisher")
        self.current_velocity = 0.0
        self.target_accel = 0.0

        self.sub = self.create_subscription(
            Float32Stamped, "/vehicle/tester/accel", self.on_accel, 1
        )
        self.sub = self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.on_twist, 1
        )

        self.pub_control_cmd = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd", 1
        )
        self.pub_gear_cmd = self.create_publisher(GearCommand, "/control/command/gear_cmd", 1)
        self.pub_odometry_cmd = self.create_publisher(Odometry, "/localization/kinematic_state", 1)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.on_timer)

    def on_accel(self, msg):
        self.target_accel = msg.data
        print(f"Set target accel: {self.target_accel}")

    def on_twist(self, msg):
        self.current_velocity = msg.longitudinal_velocity

    def on_timer(self):
        msg_control_cmd = AckermannControlCommand()
        msg_control_cmd.stamp = self.get_clock().now().to_msg()
        msg_control_cmd.lateral.steering_tire_angle = 0.0
        msg_control_cmd.lateral.steering_tire_rotation_rate = 0.0
        msg_control_cmd.longitudinal.acceleration = self.target_accel
        msg_control_cmd.longitudinal.speed = 0.0
        msg_control_cmd.longitudinal.jerk = 0.0
        self.pub_control_cmd.publish(msg_control_cmd)

        msg_gear_cmd = GearCommand()
        msg_gear_cmd.stamp = self.get_clock().now().to_msg()
        msg_gear_cmd.command = GearCommand.DRIVE
        self.pub_gear_cmd.publish(msg_gear_cmd)

        msg_odometry_cmd = Odometry()
        msg_odometry_cmd.header.stamp = self.get_clock().now().to_msg()
        msg_odometry_cmd.twist.twist.linear.x = self.current_velocity
        self.pub_odometry_cmd.publish(msg_odometry_cmd)

        print(f"publish VehicleCommand with acceleration value: {self.target_accel}")


def main(args=None):
    rclpy.init(args=args)

    accel_publisher = AccelPublisher()

    rclpy.spin(accel_publisher)

    accel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
