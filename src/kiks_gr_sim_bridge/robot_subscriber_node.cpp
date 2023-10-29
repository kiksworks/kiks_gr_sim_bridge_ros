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
  const RobotInfo & robot_info,
  const rclcpp::NodeOptions & options)
: RobotSubscriberNode(robot_info,
    std::make_shared<rclcpp::Node>(this->default_name(), options))
{
}

RobotSubscriberNode::RobotSubscriberNode(
  const RobotInfo & robot_info,
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: RobotSubscriberNode(robot_info,
    std::make_shared<rclcpp::Node>(node_name, options))
{
}

RobotSubscriberNode::RobotSubscriberNode(
  const RobotInfo & robot_info,
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: RobotSubscriberNode(robot_info,
    std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

RobotSubscriberNode::RobotSubscriberNode(
  const RobotInfo & robot_info,
  rclcpp::Node::SharedPtr node)
: ExpandedSubNode(std::move(node)),
  robot_info_(robot_info)
{
  // Parameter of vel valid duration[s]
  this->add_param<double>(
    "vel_valid_duration", 0.5, [this](double duration) {
      vel_valid_duration_ =
      std::chrono::nanoseconds(static_cast<std::size_t>(duration * 1000 * 1000 * 1000));
    });
  // Parameter of kick valid duration[s]
  this->add_param<double>(
    "kick_valid_duration", 0.5, [this](double duration) {
      kick_valid_duration_ =
      std::chrono::nanoseconds(static_cast<std::size_t>(duration * 1000 * 1000 * 1000));
    });
  // Parameter of chip kick angle[deg]
  this->add_param<double>(
    "chip_kick_deg", 60, [this](double deg) {
      const auto angle = deg * (std::acos(-1.0) / 180.0);
      chip_kick_coef_x_ = std::cos(angle);
      chip_kick_coef_z_ = std::sin(angle);
    });
  // Initialize command
  robot_info_.command->set_kickspeedx(0);
  robot_info_.command->set_kickspeedz(0);
  robot_info_.command->set_veltangent(0);
  robot_info_.command->set_velnormal(0);
  robot_info_.command->set_velangular(0);
  robot_info_.command->set_spinner(false);
  robot_info_.command->set_wheelsspeed(false);
  // Initialize subscriptions
  cmd_vel_subscription_ = (*this)->create_subscription<TwistMsg>(
    "cmd_vel",
    this->get_dynamic_qos(),
    std::bind(&RobotSubscriberNode::subscribe_cmd_vel, this, std::placeholders::_1));
  cmd_flat_kick_subscription_ = (*this)->create_subscription<JointMsg>(
    "cmd_flat_kick",
    this->get_dynamic_qos(),
    std::bind(&RobotSubscriberNode::subscribe_cmd_flat_kick, this, std::placeholders::_1));
  cmd_chip_kick_subscription_ = (*this)->create_subscription<JointMsg>(
    "cmd_chip_kick",
    this->get_dynamic_qos(),
    std::bind(&RobotSubscriberNode::subscribe_cmd_chip_kick, this, std::placeholders::_1));
  cmd_spinner_subscription_ = (*this)->create_subscription<JointMsg>(
    "cmd_spinner",
    this->get_dynamic_qos(),
    std::bind(&RobotSubscriberNode::subscribe_cmd_spinner, this, std::placeholders::_1));
  initialpose_subscription_ = (*this)->create_subscription<PoseMsg>(
    "initialpose",
    this->get_dynamic_qos(),
    std::bind(&RobotSubscriberNode::subscribe_initialpose, this, std::placeholders::_1));
  vel_valid_time_ = kick_valid_time_ = (*this)->now();
}

void RobotSubscriberNode::update_validity(const rclcpp::Time & now)
{
  if (now > vel_valid_time_) {
    robot_info_.command->set_veltangent(0);
    robot_info_.command->set_velnormal(0);
    robot_info_.command->set_velangular(0);
  }
  if (now > kick_valid_time_) {
    robot_info_.command->set_kickspeedx(0);
    robot_info_.command->set_kickspeedz(0);
  }
}

void RobotSubscriberNode::subscribe_cmd_vel(TwistMsg::ConstSharedPtr cmd_vel_msg)
{
  vel_valid_time_ = (*this)->now() + vel_valid_duration_;
  robot_info_.command->set_veltangent(cmd_vel_msg->linear.x);
  robot_info_.command->set_velnormal(cmd_vel_msg->linear.y);
  robot_info_.command->set_velangular(cmd_vel_msg->angular.z);
}

void RobotSubscriberNode::subscribe_cmd_flat_kick(JointMsg::ConstSharedPtr cmd_flat_kick_msg)
{
  kick_valid_time_ = (*this)->now() + kick_valid_duration_;
  robot_info_.command->set_kickspeedx(cmd_flat_kick_msg->velocity);
  robot_info_.command->set_kickspeedz(0);
}

void RobotSubscriberNode::subscribe_cmd_chip_kick(JointMsg::ConstSharedPtr cmd_chip_kick_msg)
{
  kick_valid_time_ = (*this)->now() + kick_valid_duration_;
  robot_info_.command->set_kickspeedx(cmd_chip_kick_msg->velocity * chip_kick_coef_x_);
  robot_info_.command->set_kickspeedz(cmd_chip_kick_msg->velocity * chip_kick_coef_z_);
}


void RobotSubscriberNode::subscribe_cmd_spinner(JointMsg::ConstSharedPtr cmd_spinner_msg)
{
  robot_info_.command->set_spinner(cmd_spinner_msg->velocity > 0);
}

void RobotSubscriberNode::subscribe_initialpose(PoseMsg::ConstSharedPtr initialpose_msg)
{
  auto robot = robot_info_.replacement->add_robots();
  const auto & pose = initialpose_msg->pose.pose;
  robot->set_x(pose.position.x);
  robot->set_y(pose.position.y);
  robot->set_dir(
    std::atan2(
      pose.orientation.z,
      pose.orientation.w) * (2 * 180.0 / std::acos(-1.0)));
  robot->set_yellowteam(robot_info_.team_is_yellow);
  robot->set_id(robot_info_.robot_id);
  *robot_info_.has_replacement = true;
}

}  // namespace kiks::gr_sim_bridge
