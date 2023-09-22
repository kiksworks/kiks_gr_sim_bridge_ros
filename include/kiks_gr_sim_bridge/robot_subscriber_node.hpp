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


#ifndef KIKS_GR_SIM_BRIDGE__ROBOT_SUBSCRIBER_NODE_HPP_
#define KIKS_GR_SIM_BRIDGE__ROBOT_SUBSCRIBER_NODE_HPP_

#include <chrono>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"

#include "kiks_gr_sim_bridge/ros_node_base.hpp"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"

namespace kiks::gr_sim_bridge
{

class RobotSubscriberNode : public RosNodeBase
{
public:
  struct RobotInfo
  {
    grSim_Robot_Command * command;
    grSim_Replacement * replacement;
    bool team_is_yellow;
    int robot_id;
  };

  inline static std::string default_name() {return "gr_sim_bridge_robot_subuscriber";}

  explicit RobotSubscriberNode(
    const RobotInfo& robot_info,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RobotSubscriberNode(
    const RobotInfo& robot_info,
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RobotSubscriberNode(
    const RobotInfo& robot_info,
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit RobotSubscriberNode(
    const RobotInfo& robot_info, rclcpp::Node::SharedPtr node);

  inline void update_validity() {update_validity(node_->now());}

  void update_validity(const rclcpp::Time & now);

private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using JointMsg = pendulum_msgs::msg::JointState;
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

  void subscribe_cmd_vel(TwistMsg::ConstSharedPtr cmd_vel_msg);

  void subscribe_cmd_flat_kick(JointMsg::ConstSharedPtr cmd_flat_kick_msg);

  void subscribe_cmd_chip_kick(JointMsg::ConstSharedPtr cmd_chip_kick_msg);

  void subscribe_cmd_spinner(JointMsg::ConstSharedPtr cmd_spinner_msg);

  void subscribe_initialpose(PoseMsg::ConstSharedPtr initialpose_msg);

  const RobotInfo robot_info_;
  double chip_kick_coef_x_, chip_kick_coef_z_;
  std::chrono::nanoseconds vel_valid_duration_, kick_valid_duration_;
  rclcpp::Time vel_valid_time_, kick_valid_time_;
  rclcpp::Subscription<TwistMsg>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<JointMsg>::SharedPtr cmd_flat_kick_subscription_,
    cmd_chip_kick_subscription_, cmd_spinner_subscription_;
  rclcpp::Subscription<PoseMsg>::SharedPtr initialpose_subscription_;
};

}  // namespace kiks::gr_sim_bridge

#endif  // KIKS_GR_SIM_BRIDGE__ROBOT_SUBSCRIBER_NODE_HPP_
