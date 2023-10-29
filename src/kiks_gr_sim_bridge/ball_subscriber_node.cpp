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


#include "kiks_gr_sim_bridge/ball_subscriber_node.hpp"

namespace kiks::gr_sim_bridge
{

BallSubscriberNode::BallSubscriberNode(
  const BallInfo & ball_info,
  const rclcpp::NodeOptions & options)
: BallSubscriberNode(ball_info,
    std::make_shared<rclcpp::Node>(this->default_name(), options))
{
}

BallSubscriberNode::BallSubscriberNode(
  const BallInfo & ball_info,
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: BallSubscriberNode(ball_info,
    std::make_shared<rclcpp::Node>(node_name, options))
{
}

BallSubscriberNode::BallSubscriberNode(
  const BallInfo & ball_info,
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: BallSubscriberNode(ball_info,
    std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

BallSubscriberNode::BallSubscriberNode(
  const BallInfo & ball_info,
  rclcpp::Node::SharedPtr node)
: ExpandedSubNode(std::move(node)),
  ball_info_(ball_info)
{
  // Initialize subscription
  initialpose_subscription_ = (*this)->create_subscription<PoseMsg>(
    "initialpose",
    this->get_dynamic_qos(),
    std::bind(&BallSubscriberNode::subscribe_initialpose, this, std::placeholders::_1));
}

void BallSubscriberNode::subscribe_initialpose(PoseMsg::ConstSharedPtr initialpose_msg)
{
  auto ball = ball_info_.replacement->mutable_ball();
  const auto & pose = initialpose_msg->pose.pose;
  ball->set_x(pose.position.x);
  ball->set_y(pose.position.y);
  ball->set_vx(0);
  ball->set_vy(0);
  *ball_info_.has_replacement = true;
}

}  // namespace kiks::gr_sim_bridge
