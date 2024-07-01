// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GSM8_CAN_INTERFACE__GSM8_CAN_INTERFACE_HPP_
#define GSM8_CAN_INTERFACE__GSM8_CAN_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>
#include <gsm8_interface_msgs/msg/b0_command.hpp>
#include <gsm8_interface_msgs/msg/b0_status.hpp>
#include <gsm8_interface_msgs/msg/b1_command.hpp>
#include <gsm8_interface_msgs/msg/b1_status.hpp>
#include <gsm8_interface_msgs/msg/m0_command.hpp>
#include <gsm8_interface_msgs/msg/m0_status.hpp>
#include <gsm8_interface_msgs/msg/m1_command.hpp>
#include <gsm8_interface_msgs/msg/m1_status.hpp>
#include <gsm8_interface_msgs/msg/m2_command.hpp>
#include <gsm8_interface_msgs/msg/m2_status.hpp>
#include <gsm8_interface_msgs/msg/m3_status.hpp>
#include <gsm8_interface_msgs/msg/m4_status.hpp>
#include <gsm8_interface_msgs/msg/m5_status.hpp>

#include <string>

class GSM8CanInterface : public rclcpp::Node
{
public:
  explicit GSM8CanInterface(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  std::string frame_id_;

  // Subscriber
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::B0Command>::SharedPtr b0_command_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::B1Command>::SharedPtr b1_command_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::M0Command>::SharedPtr m0_command_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::M1Command>::SharedPtr m1_command_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::M2Command>::SharedPtr m2_command_sub_;

  // Publisher
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::B0Status>::SharedPtr b0_status_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::B1Status>::SharedPtr b1_status_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::M0Status>::SharedPtr m0_status_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::M1Status>::SharedPtr m1_status_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::M2Status>::SharedPtr m2_status_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::M3Status>::SharedPtr m3_status_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::M4Status>::SharedPtr m4_status_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::M5Status>::SharedPtr m5_status_pub_;

  void onCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg);
  void onB0Command(const gsm8_interface_msgs::msg::B0Command::ConstSharedPtr msg);
  void onB1Command(const gsm8_interface_msgs::msg::B1Command::ConstSharedPtr msg);
  void onM0Command(const gsm8_interface_msgs::msg::M0Command::ConstSharedPtr msg);
  void onM1Command(const gsm8_interface_msgs::msg::M1Command::ConstSharedPtr msg);
  void onM2Command(const gsm8_interface_msgs::msg::M2Command::ConstSharedPtr msg);
};

#endif  // GSM8_CAN_INTERFACE__GSM8_CAN_INTERFACE_HPP_
