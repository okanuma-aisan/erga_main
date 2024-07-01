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

#ifndef GSM8_INTERFACE__GSM8_INTERFACE_HPP_
#define GSM8_INTERFACE__GSM8_INTERFACE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <signal_processing/lowpass_filter_1d.hpp>
#include <tier4_api_utils/types/response.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
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
#include <std_msgs/msg/bool.hpp>
#include <tier4_api_msgs/msg/door_status.hpp>
#include <tier4_external_api_msgs/srv/set_door.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

#include <deque>
#include <memory>
#include <string>

class GSM8Interface : public rclcpp::Node
{
public:
  explicit GSM8Interface(const rclcpp::NodeOptions & node_options);

  struct CountStamped
  {
    rclcpp::Time stamp;
    uint8_t count;
  };

private:
  // Subscriber
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    vehicle_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr
    actuation_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr shift_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr
    emergency_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_signal_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    hazard_light_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_sub_;

  rclcpp::Subscription<gsm8_interface_msgs::msg::B0Status>::SharedPtr b0_status_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::B1Status>::SharedPtr b1_status_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::M0Status>::SharedPtr m0_status_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::M1Status>::SharedPtr m1_status_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::M2Status>::SharedPtr m2_status_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::M3Status>::SharedPtr m3_status_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::M4Status>::SharedPtr m4_status_sub_;
  rclcpp::Subscription<gsm8_interface_msgs::msg::M5Status>::SharedPtr m5_status_sub_;

  // Publisher
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr shift_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_signal_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    hazard_light_status_pub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr
    actuation_status_pub_;
  rclcpp::Publisher<tier4_api_msgs::msg::DoorStatus>::SharedPtr door_status_pub_;

  rclcpp::Publisher<gsm8_interface_msgs::msg::B0Command>::SharedPtr b0_command_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::B1Command>::SharedPtr b1_command_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::M0Command>::SharedPtr m0_command_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::M1Command>::SharedPtr m1_command_pub_;
  rclcpp::Publisher<gsm8_interface_msgs::msg::M2Command>::SharedPtr m2_command_pub_;

  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr
    vehicle_twist_raw_pub_;

  // Service
  rclcpp::Service<tier4_external_api_msgs::srv::SetDoor>::SharedPtr set_door_srv_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter
  gsm8_interface_msgs::msg::B0Command b0_command_msg_;
  gsm8_interface_msgs::msg::B1Command b1_command_msg_;
  gsm8_interface_msgs::msg::M0Command m0_command_msg_;
  gsm8_interface_msgs::msg::M1Command m1_command_msg_;
  gsm8_interface_msgs::msg::M2Command m2_command_msg_;

  double loop_rate_;
  double wheel_base_;
  autoware_auto_vehicle_msgs::msg::VelocityReport current_twist_;

  double gain_LPF_vehicle_twist_;
  std::shared_ptr<LowpassFilter1d> lpf_vehicle_twist_;

  double timeout_heartbeat_;
  size_t heartbeat_window_size_;
  double heartbeat_rate_error_threshold_;
  double missing_heartbeat_error_threshold_;
  double speed_scale_factor_;
  double steering_offset_;
  uint8_t heartbeat_count_;
  rclcpp::Time heartbeat_received_time_{0, 0, RCL_ROS_TIME};
  std::deque<CountStamped> heartbeat_buffer_;

  void updateHeatbeat();
  void publishCommands();
  bool convertShiftEpb(
    const autoware_auto_vehicle_msgs::msg::GearCommand & aw_sft,
    gsm8_interface_msgs::msg::SFTCommand & gsm8_sft,
    gsm8_interface_msgs::msg::EPBCommand & gsm8_epb);

  void onTimer();

  void onVehicleCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onActuationCmd(const tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr msg);
  void onShiftCmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void onEmergencyCmd(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
  void onTurnSignalCmd(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  void onHazardLightCmd(
    const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
  void onEngage(const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg);

  void onB0Status(const gsm8_interface_msgs::msg::B0Status::ConstSharedPtr msg);
  void onB1Status(const gsm8_interface_msgs::msg::B1Status::ConstSharedPtr msg);
  void onM0Status(const gsm8_interface_msgs::msg::M0Status::ConstSharedPtr msg);
  void onM1Status(const gsm8_interface_msgs::msg::M1Status::ConstSharedPtr msg);
  void onM2Status(const gsm8_interface_msgs::msg::M2Status::ConstSharedPtr msg);
  void onM3Status(const gsm8_interface_msgs::msg::M3Status::ConstSharedPtr msg);
  void onM4Status(const gsm8_interface_msgs::msg::M4Status::ConstSharedPtr msg);
  void onM5Status(const gsm8_interface_msgs::msg::M5Status::ConstSharedPtr msg);

  void onSetDoor(
    const tier4_external_api_msgs::srv::SetDoor::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::SetDoor::Response::SharedPtr response);

  // Diagnostics
  void setupDiagnosticUpdater();
  void checkVehicleErrors(diagnostic_updater::DiagnosticStatusWrapper & stat);
  double calculateMissingHeartbeatRate(const std::deque<CountStamped> & heartbeat_buffer) const;

  // Diagnostic Updater
  diagnostic_updater::Updater diagnostic_updater_{this};
};

#endif  // GSM8_INTERFACE__GSM8_INTERFACE_HPP_
