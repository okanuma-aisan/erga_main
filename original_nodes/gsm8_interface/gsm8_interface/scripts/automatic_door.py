#!/usr/bin/env python3

# Copyright 2022 Tier IV, Inc. All rights reserved.
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
from tier4_api_msgs.msg import AwapiAutowareStatus
from tier4_api_msgs.msg import DoorStatus
from tier4_external_api_msgs.srv import SetDoor


class AutomaticDoorNode(Node):
    def __init__(self):
        super().__init__("automatic_door")
        self._autoware_status_sub = self.create_subscription(
            AwapiAutowareStatus, "/awapi/autoware/get/status", self._on_autoware_status, 10
        )

        self._door_status_sub = self.create_subscription(
            DoorStatus, "/awapi/vehicle/get/door", self._on_door_status, 10
        )

        self.door_client = self.create_client(SetDoor, "/api/external/set/door")

        self._arrived_goal = True

    def _door_callback(self, future):
        result = future.result()
        if result is not None:
            self.get_logger().info("Response received")
        else:
            self.get_logger().error(f"Exception for service: {future.exception()}")

    def _on_door_status(self, msg):
        pass

    def _on_autoware_status(self, msg):
        arrived_goal = msg.arrived_goal
        if not self._arrived_goal and arrived_goal:
            self.get_logger().info("Arrived! -> Request door opening...")
            future = self.door_client.call_async(SetDoor.Request(open=True))
            future.add_done_callback(self._door_callback)

        self._arrived_goal = arrived_goal


def main(args=None):
    rclpy.init(args=args)

    node = AutomaticDoorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
