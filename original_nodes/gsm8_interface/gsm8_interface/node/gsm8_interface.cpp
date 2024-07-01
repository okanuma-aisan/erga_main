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

#include "gsm8_interface/gsm8_interface.hpp"

#include <memory>
#include <utility>
#include <vector>

GSM8Interface::GSM8Interface(const rclcpp::NodeOptions & node_options)
: Node("gsm8_interface", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Parameter
  loop_rate_ = declare_parameter("loop_rate", 50.0);
  wheel_base_ = declare_parameter("wheel_base", 2.85);
  gain_LPF_vehicle_twist_ = declare_parameter("gain_LPF_vehicle_twist", 0.0);
  timeout_heartbeat_ = declare_parameter("timeout_heartbeat", 0.5);
  heartbeat_window_size_ = declare_parameter("heartbeat_window_size", 1000);
  heartbeat_rate_error_threshold_ = declare_parameter("heartbeat_rate_error_threshold", 35);
  missing_heartbeat_error_threshold_ = declare_parameter("missing_heartbeat_error_threshold", 2.0);
  speed_scale_factor_ = declare_parameter("speed_scale_factor", 1.0);
  steering_offset_ = declare_parameter("steering_offset", 0.0);

  // Subscriber
  vehicle_cmd_sub_ =
    this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", 1, std::bind(&GSM8Interface::onVehicleCmd, this, _1));
  shift_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&GSM8Interface::onShiftCmd, this, _1));
  emergency_cmd_sub_ = this->create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1, std::bind(&GSM8Interface::onEmergencyCmd, this, _1));
  actuation_cmd_sub_ = this->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
    "/control/command/actuation_cmd", 1, std::bind(&GSM8Interface::onActuationCmd, this, _1));
  turn_signal_cmd_sub_ =
    this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", 1,
      std::bind(&GSM8Interface::onTurnSignalCmd, this, _1));
  hazard_light_cmd_sub_ =
    this->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", 1,
      std::bind(&GSM8Interface::onHazardLightCmd, this, _1));
  engage_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::Engage>(
    "/vehicle/engage", 1, std::bind(&GSM8Interface::onEngage, this, _1));

  b0_status_sub_ = this->create_subscription<gsm8_interface_msgs::msg::B0Status>(
    "can/status/b0_status", 1, std::bind(&GSM8Interface::onB0Status, this, _1));
  b1_status_sub_ = this->create_subscription<gsm8_interface_msgs::msg::B1Status>(
    "can/status/b1_status", 1, std::bind(&GSM8Interface::onB1Status, this, _1));
  m0_status_sub_ = this->create_subscription<gsm8_interface_msgs::msg::M0Status>(
    "can/status/m0_status", 1, std::bind(&GSM8Interface::onM0Status, this, _1));
  m1_status_sub_ = this->create_subscription<gsm8_interface_msgs::msg::M1Status>(
    "can/status/m1_status", 1, std::bind(&GSM8Interface::onM1Status, this, _1));
  m2_status_sub_ = this->create_subscription<gsm8_interface_msgs::msg::M2Status>(
    "can/status/m2_status", 1, std::bind(&GSM8Interface::onM2Status, this, _1));
  m3_status_sub_ = this->create_subscription<gsm8_interface_msgs::msg::M3Status>(
    "can/status/m3_status", 1, std::bind(&GSM8Interface::onM3Status, this, _1));
  m4_status_sub_ = this->create_subscription<gsm8_interface_msgs::msg::M4Status>(
    "can/status/m4_status", 1, std::bind(&GSM8Interface::onM4Status, this, _1));
  m5_status_sub_ = this->create_subscription<gsm8_interface_msgs::msg::M5Status>(
    "can/status/m5_status", 1, std::bind(&GSM8Interface::onM5Status, this, _1));

  // Publisher
  control_mode_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", 10);
  vehicle_twist_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 10);
  steering_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", 10);
  shift_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", 10);
  turn_signal_status_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", 10);
  hazard_light_status_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
      "/vehicle/status/hazard_lights_status", 10);
  actuation_status_pub_ = this->create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
    "/vehicle/status/actuation_status", 1);
  door_status_pub_ =
    this->create_publisher<tier4_api_msgs::msg::DoorStatus>("/vehicle/status/door_status", 1);

  b0_command_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::B0Command>("can/command/b0_command", 10);
  b1_command_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::B1Command>("can/command/b1_command", 10);
  m0_command_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::M0Command>("can/command/m0_command", 10);
  m1_command_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::M1Command>("can/command/m1_command", 10);
  m2_command_pub_ =
    this->create_publisher<gsm8_interface_msgs::msg::M2Command>("can/command/m2_command", 10);

  vehicle_twist_raw_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "debug/vehicle/status/raw_twist", 10);
  lpf_vehicle_twist_ = std::make_shared<LowpassFilter1d>(gain_LPF_vehicle_twist_);
  lpf_vehicle_twist_->reset(0.0);

  // Service
  set_door_srv_ = this->create_service<tier4_external_api_msgs::srv::SetDoor>(
    "/api/vehicle/set/door", std::bind(&GSM8Interface::onSetDoor, this, _1, _2));
  // Timer
  auto on_timer = std::bind(&GSM8Interface::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / loop_rate_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer)>>(
    this->get_clock(), period, std::move(on_timer), this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  // TODO(aohsato): Create interface to/from autoware
  b0_command_msg_.hrn.data = gsm8_interface_msgs::msg::HRNCommand::OFF;
  b0_command_msg_.lgt.data = gsm8_interface_msgs::msg::LGTCommand::OFF;
  b0_command_msg_.dor.data = gsm8_interface_msgs::msg::DORCommand::FREE;
  b1_command_msg_.pow.data = gsm8_interface_msgs::msg::POWCommand::ON;
  m1_command_msg_.splim = 30.0;

  // Diagnostics
  setupDiagnosticUpdater();
}

void GSM8Interface::onTimer()
{
  updateHeatbeat();
  publishCommands();
}

void GSM8Interface::updateHeatbeat() { m0_command_msg_.heat++; }

void GSM8Interface::publishCommands()
{
  b0_command_pub_->publish(b0_command_msg_);
  b1_command_pub_->publish(b1_command_msg_);
  m0_command_pub_->publish(m0_command_msg_);
  m1_command_pub_->publish(m1_command_msg_);
  m2_command_pub_->publish(m2_command_msg_);
}

bool GSM8Interface::convertShiftEpb(
  const autoware_auto_vehicle_msgs::msg::GearCommand & aw_sft,
  gsm8_interface_msgs::msg::SFTCommand & gsm8_sft, gsm8_interface_msgs::msg::EPBCommand & gsm8_epb)
{
  bool result = false;
  switch (aw_sft.command) {
    // case autoware_auto_vehicle_msgs::msg::GearCommand::NONE:
    //   // do nothing
    //   result = true;
    //   break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::PARK:
      gsm8_sft.data = gsm8_interface_msgs::msg::SFTCommand::N;
      gsm8_epb.data = gsm8_interface_msgs::msg::EPBCommand::ON;
      result = true;
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE:
      gsm8_sft.data = gsm8_interface_msgs::msg::SFTCommand::R;
      gsm8_epb.data = gsm8_interface_msgs::msg::EPBCommand::OFF;
      result = true;
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL:
      gsm8_sft.data = gsm8_interface_msgs::msg::SFTCommand::N;
      gsm8_epb.data = gsm8_interface_msgs::msg::EPBCommand::OFF;
      result = true;
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE:
      gsm8_sft.data = gsm8_interface_msgs::msg::SFTCommand::D;
      gsm8_epb.data = gsm8_interface_msgs::msg::EPBCommand::OFF;
      result = true;
      break;
    case autoware_auto_vehicle_msgs::msg::GearCommand::LOW:
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Value error: LOW shift is unsupported");
      break;
    default:
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Value error: shift = %d", aw_sft.command);
      break;
  }

  return result;
}

void GSM8Interface::onVehicleCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  m0_command_msg_.header.stamp = msg->stamp;
  m1_command_msg_.header.stamp = msg->stamp;

  m0_command_msg_.acc = msg->longitudinal.acceleration;              // [m/s^2]
  m0_command_msg_.str = msg->lateral.steering_tire_angle;            // [rad]
  m0_command_msg_.strsp = msg->lateral.steering_tire_rotation_rate;  // [rad/s]
}

void GSM8Interface::onShiftCmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  convertShiftEpb(*msg, m1_command_msg_.sft, m1_command_msg_.epb);
}

void GSM8Interface::onEmergencyCmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  if (msg->emergency) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Emergency! But do nothing now...");
  }
}

void GSM8Interface::onActuationCmd(
  const tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr msg)
{
  m0_command_msg_.header = msg->header;
  m1_command_msg_.header = msg->header;
  m2_command_msg_.header = msg->header;

  m2_command_msg_.trtl = msg->actuation.accel_cmd;  // [-]
  m2_command_msg_.brkp = msg->actuation.brake_cmd;  // [-]
  m2_command_msg_.strv = msg->actuation.steer_cmd;  // [V]
}

void GSM8Interface::onTurnSignalCmd(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  b0_command_msg_.header.stamp = msg->stamp;

  switch (msg->command) {
    case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND:
      b0_command_msg_.bln.data = gsm8_interface_msgs::msg::BLNCommand::OFF;
      break;
    case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE:
      b0_command_msg_.bln.data = gsm8_interface_msgs::msg::BLNCommand::OFF;
      break;
    case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT:
      b0_command_msg_.bln.data = gsm8_interface_msgs::msg::BLNCommand::LEFT;
      break;
    case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT:
      b0_command_msg_.bln.data = gsm8_interface_msgs::msg::BLNCommand::RIGHT;
      break;
    default:
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Value error: turn_signal = %d", msg->command);
      break;
  }
}

void GSM8Interface::onHazardLightCmd(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  b0_command_msg_.header.stamp = msg->stamp;

  switch (msg->command) {
    case autoware_auto_vehicle_msgs::msg::HazardLightsCommand::NO_COMMAND:
      if (
        b0_command_msg_.bln.data != gsm8_interface_msgs::msg::BLNCommand::LEFT &&
        b0_command_msg_.bln.data != gsm8_interface_msgs::msg::BLNCommand::RIGHT) {
        b0_command_msg_.bln.data = gsm8_interface_msgs::msg::BLNCommand::OFF;
      }
      break;
    case autoware_auto_vehicle_msgs::msg::HazardLightsCommand::DISABLE:
      if (
        b0_command_msg_.bln.data != gsm8_interface_msgs::msg::BLNCommand::LEFT &&
        b0_command_msg_.bln.data != gsm8_interface_msgs::msg::BLNCommand::RIGHT) {
        b0_command_msg_.bln.data = gsm8_interface_msgs::msg::BLNCommand::OFF;
      }
      break;
    case autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ENABLE:
      b0_command_msg_.bln.data = gsm8_interface_msgs::msg::BLNCommand::HAZARD;
      break;
    default:
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Value error: hazard_signal = %d", msg->command);
      break;
  }
}

void GSM8Interface::onEngage(const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
  m0_command_msg_.header.stamp = this->now();

  // TODO(aohsato): Handle override status

  if (msg->engage) {
    m0_command_msg_.bwm.data = gsm8_interface_msgs::msg::BWMCommand::THROTTLE;
  } else {
    m0_command_msg_.bwm.data = gsm8_interface_msgs::msg::BWMCommand::OFF;
    b0_command_msg_.dor.data = gsm8_interface_msgs::msg::DORCommand::FREE;
  }
}

void GSM8Interface::onB0Status(const gsm8_interface_msgs::msg::B0Status::ConstSharedPtr msg)
{
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_signal_msg;
  autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_light_msg;
  turn_signal_msg.stamp = msg->header.stamp;
  hazard_light_msg.stamp = msg->header.stamp;

  switch (msg->bln.data) {
    case gsm8_interface_msgs::msg::BLNStatus::OFF:
      turn_signal_msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
      break;
    case gsm8_interface_msgs::msg::BLNStatus::LEFT:
      turn_signal_msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
      break;
    case gsm8_interface_msgs::msg::BLNStatus::RIGHT:
      turn_signal_msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
      break;
    case gsm8_interface_msgs::msg::BLNStatus::HAZARD:
      hazard_light_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE;
      break;
    default:
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Value error: S_BLN = %d", msg->bln.data);
      break;
  }

  tier4_api_msgs::msg::DoorStatus door_status_msg;

  switch (msg->dor.data) {
    case gsm8_interface_msgs::msg::DORStatus::OPEN:
      door_status_msg.status = tier4_api_msgs::msg::DoorStatus::DOOR_OPENED;
      if (b0_command_msg_.dor.data == gsm8_interface_msgs::msg::DORCommand::OPEN) {
        b0_command_msg_.dor.data = gsm8_interface_msgs::msg::DORCommand::FREE;
      }
      break;
    case gsm8_interface_msgs::msg::DORStatus::CLOSE:
      door_status_msg.status = tier4_api_msgs::msg::DoorStatus::DOOR_CLOSED;
      if (b0_command_msg_.dor.data == gsm8_interface_msgs::msg::DORCommand::CLOSE) {
        b0_command_msg_.dor.data = gsm8_interface_msgs::msg::DORCommand::FREE;
      }
      break;
    case gsm8_interface_msgs::msg::DORStatus::OPENING:
      door_status_msg.status = tier4_api_msgs::msg::DoorStatus::DOOR_OPENING;
      break;
    case gsm8_interface_msgs::msg::DORStatus::CLOSING:
      door_status_msg.status = tier4_api_msgs::msg::DoorStatus::DOOR_CLOSING;
      break;
    default:
      door_status_msg.status = tier4_api_msgs::msg::DoorStatus::UNKNOWN;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Value error: S_DOR = %d", msg->dor.data);
      break;
  }

  // msg->S_HRN
  // msg->S_LGT

  turn_signal_status_pub_->publish(turn_signal_msg);
  hazard_light_status_pub_->publish(hazard_light_msg);
  door_status_pub_->publish(door_status_msg);
}

void GSM8Interface::onB1Status(const gsm8_interface_msgs::msg::B1Status::ConstSharedPtr msg)
{
  // msg->S_POW
  // msg->S_BSOC
  // msg->S_BVOL
  // msg->S_BAMP
}

void GSM8Interface::onM0Status(const gsm8_interface_msgs::msg::M0Status::ConstSharedPtr msg)
{
  // msg->S_SPINV
  // msg->S_SPINVA
  // msg->S_SPINVB
  // msg->S_HERT
  current_twist_.header = msg->header;
  current_twist_.longitudinal_velocity =
    lpf_vehicle_twist_->filter(msg->spinv) * speed_scale_factor_;
  vehicle_twist_pub_->publish(current_twist_);

  heartbeat_buffer_.push_back({this->now(), msg->hert});
  if (heartbeat_buffer_.size() > heartbeat_window_size_) {
    heartbeat_buffer_.pop_front();
  }

  // debug
  autoware_auto_vehicle_msgs::msg::VelocityReport raw_twist_msg;
  raw_twist_msg.header = msg->header;
  raw_twist_msg.longitudinal_velocity = msg->spinv * speed_scale_factor_;
  vehicle_twist_raw_pub_->publish(raw_twist_msg);
}

void GSM8Interface::onM1Status(const gsm8_interface_msgs::msg::M1Status::ConstSharedPtr msg)
{
  // current_twist_.header = msg->header;
  // current_twist_.twist.linear.x = (msg->S_SPWL + msg->S_SPWR) / 2.0 * speed_scale_factor_;
  // vehicle_twist_pub_.publish(current_twist_);
}

void GSM8Interface::onM2Status(const gsm8_interface_msgs::msg::M2Status::ConstSharedPtr msg)
{
  // msg->S_TINVT
  // msg->S_RBRK
}

void GSM8Interface::onM3Status(const gsm8_interface_msgs::msg::M3Status::ConstSharedPtr msg)
{
  const double steering_angle = msg->str + steering_offset_;
  current_twist_.header = msg->header;
  current_twist_.heading_rate =
    (current_twist_.longitudinal_velocity * std::tan(steering_angle)) / wheel_base_;

  autoware_auto_vehicle_msgs::msg::SteeringReport steering_msg;
  steering_msg.stamp = msg->header.stamp;
  steering_msg.steering_tire_angle = steering_angle;

  // msg->S_STRT
  // msg->S_TSTRT

  vehicle_twist_pub_->publish(current_twist_);
  steering_status_pub_->publish(steering_msg);
}

void GSM8Interface::onM4Status(const gsm8_interface_msgs::msg::M4Status::ConstSharedPtr msg)
{
  autoware_auto_vehicle_msgs::msg::GearReport shift_msg;
  autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;

  shift_msg.stamp = msg->header.stamp;

  switch (msg->sft.data) {
    case gsm8_interface_msgs::msg::SFTStatus::N:
      switch (msg->epb.data) {
        case gsm8_interface_msgs::msg::EPBStatus::OFF:
          shift_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::NEUTRAL;
          break;
        case gsm8_interface_msgs::msg::EPBStatus::ON:
          shift_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::PARK;
          break;
        default:
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 3000, "Value error: S_EPB = %d", msg->epb.data);
          break;
      }
      break;
    case gsm8_interface_msgs::msg::SFTStatus::D:
      shift_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::DRIVE;
      break;
    case gsm8_interface_msgs::msg::SFTStatus::R:
      shift_msg.report = autoware_auto_vehicle_msgs::msg::GearReport::REVERSE;
      break;
    default:
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Value error: S_SFT = %d", msg->sft.data);
      break;
  }

  switch (msg->bwm.data) {
    case gsm8_interface_msgs::msg::BWMStatus::OFF:
      control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
      break;
    // case gsm8_interface_msgs::msg::BWMStatus::ACCEL:
    //   control_mode_msg.mode =
    //   autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTO_PEDAL_ONLY; break;
    // case gsm8_interface_msgs::msg::BWMStatus::STEER:
    //   control_mode_msg.mode =
    //   autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTO_STEER_ONLY; break;
    case gsm8_interface_msgs::msg::BWMStatus::ALL:
      control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
      break;
    case gsm8_interface_msgs::msg::BWMStatus::THROTTLE:
      control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
      break;
    default:
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Value error: S_BWM = %d", msg->bwm.data);
      break;
  }

  // TODO(aohsato): Handle override status

  // msg->S_BLKP
  // msg->S_BLKS
  // msg->S_OW

  shift_status_pub_->publish(shift_msg);
  control_mode_pub_->publish(control_mode_msg);

  /* publish control status */
  {
    tier4_vehicle_msgs::msg::ActuationStatusStamped actuation_status;
    actuation_status.header = msg->header;
    actuation_status.status.brake_status = msg->blkp;
    actuation_status_pub_->publish(actuation_status);
  }
}

void GSM8Interface::onM5Status(const gsm8_interface_msgs::msg::M5Status::ConstSharedPtr msg)
{
  // msg->S_TRTL
}

void GSM8Interface::onSetDoor(
  const tier4_external_api_msgs::srv::SetDoor::Request::SharedPtr request,
  const tier4_external_api_msgs::srv::SetDoor::Response::SharedPtr response)
{
  if (request->open) {
    b0_command_msg_.dor.data = gsm8_interface_msgs::msg::DORCommand::OPEN;
  } else {
    b0_command_msg_.dor.data = gsm8_interface_msgs::msg::DORCommand::CLOSE;
  }
  response->status = tier4_api_utils::response_success();
}

void GSM8Interface::setupDiagnosticUpdater()
{
  diagnostic_updater_.setHardwareID("GSM8");
  diagnostic_updater_.add("vehicle_errors", this, &GSM8Interface::checkVehicleErrors);
  diagnostic_updater_.setPeriod(1.0 / loop_rate_);
}

void GSM8Interface::checkVehicleErrors(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "OK";

  if (heartbeat_buffer_.size() < 2) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000 /*ms*/, "heartbeat is not received.");
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "heartbeat is not received.";
    stat.summary(status);

    return;
  }

  // Check if heartbeat is counted
  const size_t buffer_size = heartbeat_buffer_.size();
  const int previous_heartbeat = heartbeat_buffer_.at(buffer_size - 2).count;
  const int current_heartbeat = heartbeat_buffer_.at(buffer_size - 1).count;
  if (current_heartbeat == previous_heartbeat) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000 /*ms*/, "Duplicate heartbeat counts.");
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Duplicate heartbeat counts.";
  }

  // Calculate average rate of heartbeat (Hz)
  const double time_from_first_to_last =
    (heartbeat_buffer_.back().stamp - heartbeat_buffer_.front().stamp).seconds();
  const double heartbeat_rate = 1.0 / (time_from_first_to_last / heartbeat_buffer_.size());
  if (heartbeat_rate < heartbeat_rate_error_threshold_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000 /*ms*/, "heartbeat rate is low.");
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "heartbeat rate is low.";
  }

  // Calculate missing heartbeat rate (%)
  const double missing_heartbeat_rate = calculateMissingHeartbeatRate(heartbeat_buffer_);
  if (missing_heartbeat_rate > missing_heartbeat_error_threshold_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000 /*ms*/,
      "missing heartbeat has been detected. missing rate is larger than error threshold.");
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message =
      "missing heartbeat has been detected. missing rate is larger than error threshold.";
  } else if (
    missing_heartbeat_rate > 0 &&  // NOLINT
    missing_heartbeat_rate <= missing_heartbeat_error_threshold_ &&
    status.level != diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000 /*ms*/,
      "missing heartbeat has been detected. missing rate is less than error threshold.");
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message =
      "missing heartbeat has been detected. missing rate is less than error threshold.";
  }

  // Check if heartbeat exceeds timeout
  const auto time_from_heartbeat = this->now() - heartbeat_buffer_.back().stamp;
  const bool exceed_timeout = time_from_heartbeat.seconds() > timeout_heartbeat_;
  if (exceed_timeout) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000 /*ms*/, "heartbeat msg timeout.");
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "heartbeat msg timeout.";
  }

  stat.addf("Heartbeat count", "%d", heartbeat_buffer_.back().count);
  stat.addf("Time from heartbeat", "%lf", time_from_heartbeat.seconds());
  stat.addf("Heartbeat rate [Hz]", "%lf", heartbeat_rate);
  stat.addf("Missing heartbeat rate [%]", "%lf", missing_heartbeat_rate);
  stat.summary(status);

  // prevent diagnostics from detecting missing heartbeats after timeout
  if (exceed_timeout) {
    heartbeat_buffer_.clear();
  }
}

double GSM8Interface::calculateMissingHeartbeatRate(
  const std::deque<CountStamped> & heartbeat_buffer) const
{
  if (heartbeat_buffer.size() < 2) {
    return 0;
  }

  // calculate sum of missing heartbeats
  int missing_heartbeat_count = 0;
  for (size_t i = 0; i < heartbeat_buffer.size() - 1; i++) {
    const int current_heartbeat = heartbeat_buffer.at(i).count;
    const int next_heartbeat = heartbeat_buffer.at(i + 1).count;
    if (current_heartbeat == next_heartbeat) {
      continue;
    }

    const bool heartbeat_jump_back = next_heartbeat < current_heartbeat;
    const size_t heartbeat_size = 0xFF + 1;
    missing_heartbeat_count += next_heartbeat - current_heartbeat - 1;
    if (heartbeat_jump_back) {
      missing_heartbeat_count += heartbeat_size;
    }
  }

  // calculate missing heartbeat rate in window size
  const double missing_heartbeat_rate = 100.0 * missing_heartbeat_count / heartbeat_buffer.size();
  return missing_heartbeat_rate;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GSM8Interface)
