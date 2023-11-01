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
  rclcpp::Node::SharedPtr node)
: ExpandedSubNode(std::move(node))
{
  this->add_param(
    "timeout_duration", 0.1, [this](double duration) {
      cmd_vel_timeout_duration_ = std::chrono::nanoseconds(std::int64_t(1e9 * duration));
      cmd_vel_timeout_callback_timer_.reset();
    });
  this->add_param(
    "timeout_callback_count", 30, [this](int count) {
      cmd_vel_timeout_callback_remaining_ = timeout_callback_count_ = count;
    });
}

}  // namespace kiks::gr_sim_bridge
