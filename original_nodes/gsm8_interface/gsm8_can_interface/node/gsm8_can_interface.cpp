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

#include "gsm8_can_interface/gsm8_can_interface.hpp"

GSM8CanInterface::GSM8CanInterface(const rclcpp::NodeOptions & node_options)
: Node("gsm8_can_interface", node_options)
{
  using std::placeholders::_1;

  frame_id_ = declare_parameter("frame_id", "base_link");

  // Subscriber
  can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    "/gsm8/from_can_bus", 10, std::bind(&GSM8CanInterface::onCanFrame, this, _1));
  b0_command_sub_ = this->create_subscription<gsm8_interface_msgs::msg::B0Command>(
    "can/command/b0_command", 10, std::bind(&GSM8CanInterface::onB0Command, this, _1));
  b1_command_sub_ = this->create_subscription<gsm8_interface_msgs::msg::B1Command>(
    "can/command/b1_command", 10, std::bind(&GSM8CanInterface::onB1Command, this, _1));
  m0_command_sub_ = this->create_subscription<gsm8_interface_msgs::msg::M0Command>(
    "can/command/m0_command", 10, std::bind(&GSM8CanInterface::onM0Command, this, _1));
  m1_command_sub_ = this->create_subscription<gsm8_interface_msgs::msg::M1Command>(
    "can/command/m1_command", 10, std::bind(&GSM8CanInterface::onM1Command, this, _1));
  m2_command_sub_ = this->create_subscription<gsm8_interface_msgs::msg::M2Command>(
    "can/command/m2_command", 10, std::bind(&GSM8CanInterface::onM2Command, this, _1));

  // Publisher
  can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/gsm8/to_can_bus", 10);
  b0_status_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::B0Status>("can/status/b0_status", 10);
  b1_status_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::B1Status>("can/status/b1_status", 10);
  m0_status_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::M0Status>("can/status/m0_status", 10);
  m1_status_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::M1Status>("can/status/m1_status", 10);
  m2_status_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::M2Status>("can/status/m2_status", 10);
  m3_status_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::M3Status>("can/status/m3_status", 10);
  m4_status_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::M4Status>("can/status/m4_status", 10);
  m5_status_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::M5Status>("can/status/m5_status", 10);
}

void GSM8CanInterface::onCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  switch (msg->id) {
    case 0x301: {
      gsm8_interface_msgs::msg::M0Status m0_status_msg;
      m0_status_msg.header.stamp = msg->header.stamp;
      m0_status_msg.header.frame_id = frame_id_;

      m0_status_msg.spinv =
        (static_cast<int16_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.010;  // [m/s]
      m0_status_msg.spinva =
        (static_cast<uint16_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.010;  // [m/s]
      m0_status_msg.spinvb =
        (static_cast<uint16_t>((msg->data.at(5) << 8) + msg->data.at(4))) * 0.010;  // [m/s]
      m0_status_msg.hert = static_cast<unsigned char>(msg->data.at(6));             // [-]

      m0_status_pub_->publish(m0_status_msg);
      break;
    }

    case 0x302: {
      gsm8_interface_msgs::msg::M1Status m1_status_msg;
      m1_status_msg.header.stamp = msg->header.stamp;
      m1_status_msg.header.frame_id = frame_id_;

      m1_status_msg.spwl =
        (static_cast<int16_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.010;  // [m/s]
      m1_status_msg.spwr =
        (static_cast<int16_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.010;  // [m/s]

      m1_status_pub_->publish(m1_status_msg);
      break;
    }

    case 0x303: {
      gsm8_interface_msgs::msg::M2Status m2_status_msg;
      m2_status_msg.header.stamp = msg->header.stamp;
      m2_status_msg.header.frame_id = frame_id_;

      m2_status_msg.tinvt =
        (static_cast<int16_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.010;  // [Nm]
      m2_status_msg.rbrk =
        (static_cast<uint16_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.010;  // [Nm]

      m2_status_pub_->publish(m2_status_msg);
      break;
    }

    case 0x304: {
      gsm8_interface_msgs::msg::M3Status m3_status_msg;
      m3_status_msg.header.stamp = msg->header.stamp;
      m3_status_msg.header.frame_id = frame_id_;

      m3_status_msg.str =
        (static_cast<int16_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.001;  // [rad]
      m3_status_msg.strt =
        (static_cast<int16_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.001;  // [Nm]
      m3_status_msg.tstrt =
        (static_cast<int16_t>((msg->data.at(5) << 8) + msg->data.at(4))) * 0.001;  // [Nm]

      m3_status_pub_->publish(m3_status_msg);
      break;
    }

    case 0x305: {
      gsm8_interface_msgs::msg::M4Status m4_status_msg;
      m4_status_msg.header.stamp = msg->header.stamp;
      m4_status_msg.header.frame_id = frame_id_;

      m4_status_msg.sft.data = static_cast<unsigned char>(msg->data.at(0));  // [-]
      m4_status_msg.epb.data = static_cast<unsigned char>(msg->data.at(1));  // [-]
      m4_status_msg.blkp =
        (static_cast<int16_t>((msg->data.at(3) << 8) + msg->data.at(2))) * 0.001;  // [MPa]
      m4_status_msg.blks =
        (static_cast<uint16_t>((msg->data.at(5) << 8) + msg->data.at(4))) * 0.001;  // [-]
      m4_status_msg.bwm.data = static_cast<unsigned char>(msg->data.at(6));         // [-]
      m4_status_msg.ow = static_cast<unsigned char>(msg->data.at(7));               // [-]

      m4_status_pub_->publish(m4_status_msg);
      break;
    }

    case 0x306: {
      gsm8_interface_msgs::msg::M5Status m5_status_msg;
      m5_status_msg.header.stamp = msg->header.stamp;
      m5_status_msg.header.frame_id = frame_id_;

      m5_status_msg.trtl =
        (static_cast<uint16_t>((msg->data.at(1) << 8) + msg->data.at(0))) * 0.001;  // [-]

      m5_status_pub_->publish(m5_status_msg);
      break;
    }

    case 0x307: {
      gsm8_interface_msgs::msg::B0Status b0_status_msg;
      b0_status_msg.header.stamp = msg->header.stamp;
      b0_status_msg.header.frame_id = frame_id_;

      b0_status_msg.bln.data = static_cast<unsigned char>(msg->data.at(0));  // [-]
      b0_status_msg.hrn.data = static_cast<unsigned char>(msg->data.at(1));  // [-]
      b0_status_msg.lgt.data = static_cast<unsigned char>(msg->data.at(2));  // [-]
      b0_status_msg.dor.data = static_cast<unsigned char>(msg->data.at(3));  // [-]

      b0_status_pub_->publish(b0_status_msg);
      break;
    }

    case 0x308: {
      gsm8_interface_msgs::msg::B1Status b1_status_msg;
      b1_status_msg.header.stamp = msg->header.stamp;
      b1_status_msg.header.frame_id = frame_id_;

      b1_status_msg.pow.data = static_cast<unsigned char>(msg->data.at(0));  // [-]
      b1_status_msg.bsoc =
        (static_cast<int16_t>((msg->data.at(2) << 8) + msg->data.at(1))) * 0.010;  // [%]
      b1_status_msg.bvol =
        (static_cast<int16_t>((msg->data.at(4) << 8) + msg->data.at(3))) * 0.010;  // [V]
      b1_status_msg.bamp =
        (static_cast<uint16_t>((msg->data.at(6) << 8) + msg->data.at(5))) * 0.010;  // [A]

      b1_status_pub_->publish(b1_status_msg);
      break;
    }

    default: {
      break;
    }
  }
}

void GSM8CanInterface::onM0Command(const gsm8_interface_msgs::msg::M0Command::ConstSharedPtr msg)
{
  can_msgs::msg::Frame can_msg;
  can_msg.header = msg->header;
  can_msg.is_rtr = false;
  can_msg.is_extended = false;
  can_msg.is_error = false;
  can_msg.dlc = 8;

  can_msg.id = 0x200;
  can_msg.data.at(0) = (static_cast<int16_t>(msg->acc / 0.010)) & 0xFF;
  can_msg.data.at(1) = (static_cast<int16_t>(msg->acc / 0.010)) >> 8;
  can_msg.data.at(2) = (static_cast<int16_t>(msg->str / 0.001)) & 0xFF;
  can_msg.data.at(3) = (static_cast<int16_t>(msg->str / 0.001)) >> 8;
  can_msg.data.at(4) = (static_cast<uint16_t>(msg->strsp / 0.001)) & 0xFF;
  can_msg.data.at(5) = (static_cast<uint16_t>(msg->strsp / 0.001)) >> 8;
  can_msg.data.at(6) = msg->bwm.data;
  can_msg.data.at(7) = msg->heat;

  can_pub_->publish(can_msg);
}

void GSM8CanInterface::onM1Command(const gsm8_interface_msgs::msg::M1Command::ConstSharedPtr msg)
{
  can_msgs::msg::Frame can_msg;
  can_msg.header = msg->header;
  can_msg.is_rtr = false;
  can_msg.is_extended = false;
  can_msg.is_error = false;
  can_msg.dlc = 8;

  can_msg.id = 0x201;
  can_msg.data.at(0) = msg->sft.data;
  can_msg.data.at(1) = msg->epb.data;
  can_msg.data.at(2) = (static_cast<int16_t>(msg->splim / 0.010)) & 0xFF;
  can_msg.data.at(3) = (static_cast<int16_t>(msg->splim / 0.010)) >> 8;
  can_msg.data.at(4) = 0;
  can_msg.data.at(5) = 0;
  can_msg.data.at(6) = 0;
  can_msg.data.at(7) = 0;

  can_pub_->publish(can_msg);
}

void GSM8CanInterface::onM2Command(const gsm8_interface_msgs::msg::M2Command::ConstSharedPtr msg)
{
  can_msgs::msg::Frame can_msg;
  can_msg.header = msg->header;
  can_msg.is_rtr = false;
  can_msg.is_extended = false;
  can_msg.is_error = false;
  can_msg.dlc = 8;

  can_msg.id = 0x202;
  can_msg.data.at(0) = (static_cast<int16_t>(msg->trtl / 0.001)) & 0xFF;
  can_msg.data.at(1) = (static_cast<int16_t>(msg->trtl / 0.001)) >> 8;
  can_msg.data.at(2) = (static_cast<int16_t>(msg->brkp / 0.010)) & 0xFF;
  can_msg.data.at(3) = (static_cast<int16_t>(msg->brkp / 0.010)) >> 8;
  can_msg.data.at(4) = (static_cast<int16_t>(msg->strv / 0.010)) & 0xFF;
  can_msg.data.at(5) = (static_cast<int16_t>(msg->strv / 0.010)) >> 8;
  can_msg.data.at(6) = 0;
  can_msg.data.at(7) = 0;

  can_pub_->publish(can_msg);
}

void GSM8CanInterface::onB0Command(const gsm8_interface_msgs::msg::B0Command::ConstSharedPtr msg)
{
  can_msgs::msg::Frame can_msg;
  can_msg.header = msg->header;
  can_msg.is_rtr = false;
  can_msg.is_extended = false;
  can_msg.is_error = false;
  can_msg.dlc = 8;

  can_msg.id = 0x203;
  can_msg.data.at(0) = msg->bln.data;
  can_msg.data.at(1) = msg->hrn.data;
  can_msg.data.at(2) = msg->lgt.data;
  can_msg.data.at(3) = msg->dor.data;
  can_msg.data.at(4) = 0;
  can_msg.data.at(5) = 0;
  can_msg.data.at(6) = 0;
  can_msg.data.at(7) = 0;

  can_pub_->publish(can_msg);
}

void GSM8CanInterface::onB1Command(const gsm8_interface_msgs::msg::B1Command::ConstSharedPtr msg)
{
  can_msgs::msg::Frame can_msg;
  can_msg.header = msg->header;
  can_msg.is_rtr = false;
  can_msg.is_extended = false;
  can_msg.is_error = false;
  can_msg.dlc = 8;

  can_msg.id = 0x204;
  can_msg.data.at(0) = msg->pow.data;
  can_msg.data.at(1) = 0;
  can_msg.data.at(2) = 0;
  can_msg.data.at(3) = 0;
  can_msg.data.at(4) = 0;
  can_msg.data.at(5) = 0;
  can_msg.data.at(6) = 0;
  can_msg.data.at(7) = 0;

  can_pub_->publish(can_msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GSM8CanInterface)
