// Copyright 2023 KIKS
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


#include "kiks_gr_sim_bridge/robot_subscriber_node.hpp"

#include <cmath>
#include <cstddef>

namespace kiks::gr_sim_bridge
{

RobotSubscriberNode::RobotSubscriberNode(
  grSim_Robot_Command * const command,
  grSim_Replacement * const replacement,
  bool team_is_yellow,
  const rclcpp::NodeOptions & options)
: RobotSubscriberNode(command, replacement, team_is_yellow,
    std::make_shared<rclcpp::Node>(this->default_name(), options))
{
}

RobotSubscriberNode::RobotSubscriberNode(
  grSim_Robot_Command * const command,
  grSim_Replacement * const replacement,
  bool team_is_yellow,
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: RobotSubscriberNode(command, replacement, team_is_yellow,
    std::make_shared<rclcpp::Node>(node_name, options))
{
}

RobotSubscriberNode::RobotSubscriberNode(
  grSim_Robot_Command * const command, grSim_Replacement * const replacement,
  bool team_is_yellow,
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: RobotSubscriberNode(command, replacement, team_is_yellow,
    std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

RobotSubscriberNode::RobotSubscriberNode(
  grSim_Robot_Command * const command,
  grSim_Replacement * const replacement,
  bool team_is_yellow,
  rclcpp::Node::SharedPtr node)
: RosNodeBase(std::move(node)),
  command_(command),
  replacement_(replacement)
{
  // Parameter of kick valid duration[s]
  this->add_parameter<double>(
    "kick_valid_duration", 0.5, [this](const auto & param) {
      kick_valid_duration_ =
      std::chrono::nanoseconds(static_cast<std::size_t>(param.as_double() * 1000 * 1000 * 1000));
    });
  // Parameter of chip kick angle[deg]
  this->add_parameter<double>(
    "chip_kick_deg", 60, [this](const auto & param) {
      const auto angle = param.as_double() * (std::acos(-1.0) / 180.0);
      chip_kick_coef_x_ = std::cos(angle);
      chip_kick_coef_z_ = std::sin(angle);
    });
  // Initialize command
  command_->set_kickspeedx(0);
  command_->set_kickspeedz(0);
  command_->set_veltangent(0);
  command_->set_velnormal(0);
  command_->set_velangular(0);
  command_->set_spinner(false);
  command_->set_wheelsspeed(false);
  // Initialize subscriptions
  cmd_vel_subscription_ = node_->create_subscription<TwistMsg>(
    "cmd_vel",
    this->get_dynamic_qos(),
    std::bind(&RobotSubscriberNode::subscribe_cmd_vel, this, std::placeholders::_1));
  cmd_flat_kick_subscription_ = node_->create_subscription<JointMsg>(
    "cmd_flat_kick",
    this->get_dynamic_qos(),
    std::bind(&RobotSubscriberNode::subscribe_cmd_flat_kick, this, std::placeholders::_1));
  cmd_chip_kick_subscription_ = node_->create_subscription<JointMsg>(
    "cmd_chip_kick",
    this->get_dynamic_qos(),
    std::bind(&RobotSubscriberNode::subscribe_cmd_chip_kick, this, std::placeholders::_1));
  cmd_spinner_subscription_ = node_->create_subscription<JointMsg>(
    "cmd_spinner",
    this->get_dynamic_qos(),
    std::bind(&RobotSubscriberNode::subscribe_cmd_spinner, this, std::placeholders::_1));
  initialpose_subscription_ = node_->create_subscription<PoseMsg>(
    "initialpose",
    this->get_dynamic_qos(),
    std::bind(
      team_is_yellow ? &RobotSubscriberNode::subscribe_initialpose<true> : &
      RobotSubscriberNode::subscribe_initialpose<false>, this, std::placeholders::_1));
}

void RobotSubscriberNode::update_validity()
{
  if (node_->now() < kick_valid_time_) {
    command_->set_kickspeedx(0);
    command_->set_kickspeedz(0);
  }
}

void RobotSubscriberNode::subscribe_cmd_vel(TwistMsg::ConstSharedPtr cmd_vel_msg)
{
  command_->set_veltangent(
    cmd_vel_msg->linear.x == 0 ?
    1 : (cmd_vel_msg->linear.y / cmd_vel_msg->linear.x));
  command_->set_velnormal(std::hypot(cmd_vel_msg->linear.x, cmd_vel_msg->linear.y));
  command_->set_velangular(cmd_vel_msg->angular.z);
}

void RobotSubscriberNode::subscribe_cmd_flat_kick(JointMsg::ConstSharedPtr cmd_flat_kick_msg)
{
  kick_valid_time_ = node_->now() + kick_valid_duration_;
  command_->set_kickspeedx(cmd_flat_kick_msg->velocity);
  command_->set_kickspeedz(0);
}

void RobotSubscriberNode::subscribe_cmd_chip_kick(JointMsg::ConstSharedPtr cmd_chip_kick_msg)
{
  kick_valid_time_ = node_->now() + kick_valid_duration_;
  command_->set_kickspeedx(cmd_chip_kick_msg->velocity * chip_kick_coef_x_);
  command_->set_kickspeedz(cmd_chip_kick_msg->velocity * chip_kick_coef_z_);
}


void RobotSubscriberNode::subscribe_cmd_spinner(JointMsg::ConstSharedPtr cmd_spinner_msg)
{
  command_->set_spinner(cmd_spinner_msg->velocity > 0);
}

template<bool kTeamIsYellow>
void RobotSubscriberNode::subscribe_initialpose(PoseMsg::ConstSharedPtr initialpose_msg)
{
  auto robot = replacement_->add_robots();
  const auto & pose = initialpose_msg->pose.pose;
  robot->set_x(pose.position.x);
  robot->set_y(pose.position.y);
  robot->set_dir(std::atan2(pose.orientation.z, pose.orientation.w));
  robot->set_yellowteam(kTeamIsYellow);
}

}  // namespace kiks::gr_sim_bridge
