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
#include "kiks_gr_sim_bridge/expanded_sub_node.hpp"

namespace kiks::gr_sim_bridge
{

class BallSubscriberNode : public ExpandedSubNode
{
public:
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

  static inline constexpr auto default_name()
  {
    return "gr_sim_bridge_ball_subuscriber";
  }

  explicit BallSubscriberNode(rclcpp::Node::SharedPtr node);

  template <class... Args>
  explicit inline BallSubscriberNode(const Args... args)
  : BallSubscriberNode(std::make_shared<rclcpp::Node>(args...))
  {
  }
  
  template <class T>
  inline void set_initialpose_callback(const T & callback)
  {
    initialpose_subscription_ = (*this)->create_subscription<PoseMsg>(
      "initial_pose", this->get_dynamic_reliable_qos(), callback);
  }

private:
  rclcpp::Subscription<PoseMsg>::SharedPtr initialpose_subscription_;
};

}  // namespace kiks::gr_sim_bridge

#endif  // KIKS_GR_SIM_BRIDGE__BALL_SUBSCRIBER_NODE_HPP_
