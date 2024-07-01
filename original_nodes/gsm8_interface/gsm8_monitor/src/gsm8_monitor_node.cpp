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

#include "gsm8_monitor/gsm8_monitor_node.hpp"

namespace gsm8_monitor
{
GSM8Monitor::GSM8Monitor(const rclcpp::NodeOptions & node_options)
: Node("gsm8_monitor", node_options)
{
  using std::placeholders::_1;

  // Parameters
  params_ = getParams();

  // State
  state_ = initState();

  // Subscribers
  m0_status_sub_ = this->create_subscription<M0Status>(
    "~/input/m0status", 1, std::bind(&GSM8Monitor::onM0Status, this, _1));
  gnss_velocity_sub_ = this->create_subscription<TwistWithCovarianceStamped>(
    "~/input/gnss_velocity", 1, std::bind(&GSM8Monitor::onGnssVelocity, this, _1));

  // Diagnostics
  setupDiagnosticUpdater();
}

void GSM8Monitor::onM0Status(const M0Status::ConstSharedPtr msg) { state_.current_m0_status = msg; }

void GSM8Monitor::onGnssVelocity(const TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  state_.current_gnss_velocity = msg;

  diagnostic_updater_.force_update();
}

GSM8MonitorParameters GSM8Monitor::getParams()
{
  GSM8MonitorParameters p{};
  p.can_velocity_error_ratio_threshold =
    declare_parameter("can_velocity_error_ratio_threshold", 0.2);
  p.min_velocity_to_start_diagnosis = declare_parameter("min_velocity_to_start_diagnosis", 2.0);

  return p;
}

GSM8MonitorState GSM8Monitor::initState()
{
  GSM8MonitorState s{};
  s.current_m0_status = nullptr;
  s.current_gnss_velocity = nullptr;

  return s;
}

void GSM8Monitor::setupDiagnosticUpdater()
{
  diagnostic_updater_.setHardwareID("GSM8");
  diagnostic_updater_.add("velocity_errors", this, &GSM8Monitor::checkVelocityErrors);
}

void GSM8Monitor::checkVelocityErrors(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "OK";

  // Guard
  if (state_.current_m0_status == nullptr || state_.current_gnss_velocity == nullptr) {
    return;
  }

  const double & current_can_velocity = state_.current_m0_status->spinv;

  const double & vx = state_.current_gnss_velocity->twist.twist.linear.x;
  const double & vy = state_.current_gnss_velocity->twist.twist.linear.y;
  const double current_gnss_velocity = std::sqrt(std::pow(vx, 2.0) + std::pow(vy, 2.0));

  // Guard
  if (
    current_can_velocity < params_.min_velocity_to_start_diagnosis ||
    current_gnss_velocity < params_.min_velocity_to_start_diagnosis) {
    return;
  }

  const double ratio = std::abs(current_can_velocity) / current_gnss_velocity;
  const double min_ratio = 1.0 - params_.can_velocity_error_ratio_threshold;
  const double max_ratio = 1.0 + params_.can_velocity_error_ratio_threshold;

  if (ratio < min_ratio || max_ratio < ratio) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000 /*ms*/, "CAN velocity reliability is low");
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "CAN velocity reliability is low";
  }

  stat.addf("CAN velocity", "%f", current_can_velocity);
  stat.addf("GNSS velocity", "%f", current_gnss_velocity);
  stat.summary(status);
}

}  // namespace gsm8_monitor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gsm8_monitor::GSM8Monitor)
