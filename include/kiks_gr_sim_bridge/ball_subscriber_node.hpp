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


#ifndef KIKS_GR_SIM_BRIDGE__BALL_SUBSCRIBER_NODE_HPP_
#define KIKS_GR_SIM_BRIDGE__BALL_SUBSCRIBER_NODE_HPP_

#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "grSim_Replacement.pb.h"
#include "kiks_gr_sim_bridge/ros_node_base.hpp"

namespace kiks::gr_sim_bridge
{

class BallSubscriberNode : public RosNodeBase
{
public:
  struct BallInfo
  {
    grSim_Replacement * replacement;
  };

  inline static std::string default_name() {return "gr_sim_bridge_ball_subuscriber";}

  explicit BallSubscriberNode(
    const BallInfo& robot_info,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  BallSubscriberNode(
    const BallInfo& robot_info,
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  BallSubscriberNode(
    const BallInfo& robot_info,
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit BallSubscriberNode(
    const BallInfo& robot_info, rclcpp::Node::SharedPtr node);

private:
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

  void subscribe_initialpose(PoseMsg::ConstSharedPtr initialpose_msg);

  const BallInfo ball_info_;
  rclcpp::Subscription<PoseMsg>::SharedPtr initialpose_subscription_;
};

}  // namespace kiks::gr_sim_bridge

#endif  // KIKS_GR_SIM_BRIDGE__BALL_SUBSCRIBER_NODE_HPP_
