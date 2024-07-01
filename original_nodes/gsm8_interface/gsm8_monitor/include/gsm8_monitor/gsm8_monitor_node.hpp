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

#ifndef GSM8_MONITOR__GSM8_MONITOR_NODE_HPP_
#define GSM8_MONITOR__GSM8_MONITOR_NODE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <gsm8_interface_msgs/msg/m0_status.hpp>

namespace gsm8_monitor
{
using geometry_msgs::msg::TwistWithCovarianceStamped;
using gsm8_interface_msgs::msg::M0Status;

struct GSM8MonitorParameters
{
  double can_velocity_error_ratio_threshold;
  double min_velocity_to_start_diagnosis;
};

struct GSM8MonitorState
{
  M0Status::ConstSharedPtr current_m0_status;
  TwistWithCovarianceStamped::ConstSharedPtr current_gnss_velocity;
};

class GSM8Monitor : public rclcpp::Node
{
public:
  explicit GSM8Monitor(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  GSM8MonitorParameters params_;

  // State
  GSM8MonitorState state_;

  diagnostic_updater::Updater diagnostic_updater_{this};

  // Subscribers
  rclcpp::Subscription<M0Status>::SharedPtr m0_status_sub_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr gnss_velocity_sub_;

  // Publishers

  // Callbacks
  void onM0Status(const M0Status::ConstSharedPtr msg);
  void onGnssVelocity(const TwistWithCovarianceStamped::ConstSharedPtr msg);
  void onTimer();

  // Functions
  GSM8MonitorParameters getParams();
  GSM8MonitorState initState();
  void setupDiagnosticUpdater();
  void checkVelocityErrors(diagnostic_updater::DiagnosticStatusWrapper & stat);
};
}  // namespace gsm8_monitor

#endif  // GSM8_MONITOR__GSM8_MONITOR_NODE_HPP_
