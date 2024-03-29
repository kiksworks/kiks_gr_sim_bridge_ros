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

BallSubscriberNode::BallSubscriberNode(rclcpp::Node::SharedPtr node)
: ExpandedSubNode(std::move(node))
{
}

}  // namespace kiks::gr_sim_bridge
