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

#include "kiks_gr_sim_bridge/expanded_sub_node.hpp"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"

namespace kiks::gr_sim_bridge
{

class RobotSubscriberNode : public ExpandedSubNode
{
public:
  using TwistMsg = geometry_msgs::msg::Twist;
  using JointMsg = pendulum_msgs::msg::JointState;
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

  inline static constexpr auto default_name() noexcept
  {
    return "gr_sim_bridge_robot_subuscriber";
  }

  explicit inline  RobotSubscriberNode(
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : RobotSubscriberNode(std::make_shared<rclcpp::Node>(default_name(), "/", node_options))
  {
  }

  RobotSubscriberNode(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : RobotSubscriberNode(std::make_shared<rclcpp::Node>(node_name, "/", node_options))
  {
  }

  RobotSubscriberNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : RobotSubscriberNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, node_options))
  {
  }

  RobotSubscriberNode(rclcpp::Node::SharedPtr node);
  
  template <class T>
  inline void set_cmd_vel_call_back(const T & call_back)
  {
    call_back(TwistMsg::ConstSharedPtr());
    cmd_vel_subscription_ = (*this)->create_subscription<TwistMsg>("cmd_vel", this->get_dynamic_qos(), [call_back, this](TwistMsg::ConstSharedPtr cmd_vel) {
        call_back(cmd_vel);
        // cmd_vel_timeout_handle_timer_->reset();
      });
  }

private:
  rclcpp::Subscription<TwistMsg>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<JointMsg>::SharedPtr cmd_flat_kick_subscription_;
  rclcpp::Subscription<PoseMsg>::SharedPtr initialpose_subscription_;
};

}  // namespace kiks::gr_sim_bridge

#endif  // KIKS_GR_SIM_BRIDGE__ROBOT_SUBSCRIBER_NODE_HPP_
