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
  
  template <class T, class U>
  inline void set_cmd_vel_call_back(const T & call_back, const U timeout_callback)
  {
    auto timer_callback = [this, timeout_callback] {
        timeout_callback();
        this->cmd_vel_timeout_callback_timer_.reset();
      };

    if (cmd_vel_timeout_callback_timer_) {
      cmd_vel_timeout_callback_timer_ = (*this)->create_wall_timer(cmd_vel_timeout_duration_, timer_callback);
    }
    
    cmd_vel_subscription_ = (*this)->create_subscription<TwistMsg>("cmd_vel", this->get_dynamic_qos(), [call_back, timer_callback, this](TwistMsg::ConstSharedPtr cmd_vel) {
        call_back(cmd_vel);
        if(cmd_vel_timeout_callback_timer_) {
          cmd_vel_timeout_callback_timer_->reset();
        }
        else {
          cmd_vel_timeout_callback_timer_ = (*this)->create_wall_timer(cmd_vel_timeout_duration_, timer_callback);
        }
      });
  }
  
  template <class T>
  inline void set_cmd_vel_call_back(const T & call_back)
  {
    this->set_cmd_vel_call_back(call_back, []{});
  }
  
  template <class T>
  inline void set_cmd_spinner_callback(const T & callback)
  {
    cmd_spinner_subscription_ = (*this)->create_subscription<JointMsg>(
      "cmd_spinner", this->get_static_qos(), callback);
  }
  
  template <class T>
  inline void set_cmd_flat_kick_callback(const T & callback)
  {
    cmd_flat_kick_subscription_ = (*this)->create_subscription<JointMsg>(
      "cmd_flat_kick", this->get_dynamic_reliable_qos(), callback);
  }
  
  template <class T>
  inline void set_cmd_chip_kick_callback(const T & callback)
  {
    cmd_chip_kick_subscription_ = (*this)->create_subscription<JointMsg>(
      "cmd_chip_kick", this->get_dynamic_reliable_qos(), callback);
  }
  
  template <class T>
  inline void set_initialpose_callback(const T & callback)
  {
    initialpose_subscription_ = (*this)->create_subscription<PoseMsg>(
      "initialpose", this->get_dynamic_reliable_qos(), callback);
  }

private:
  std::chrono::nanoseconds cmd_vel_timeout_duration_;

  rclcpp::Subscription<TwistMsg>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<JointMsg>::SharedPtr cmd_spinner_subscription_, cmd_flat_kick_subscription_, cmd_chip_kick_subscription_;
  rclcpp::Subscription<PoseMsg>::SharedPtr initialpose_subscription_;
  rclcpp::TimerBase::SharedPtr cmd_vel_timeout_callback_timer_;
};

}  // namespace kiks::gr_sim_bridge

#endif  // KIKS_GR_SIM_BRIDGE__ROBOT_SUBSCRIBER_NODE_HPP_
