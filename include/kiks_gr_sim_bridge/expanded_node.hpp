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


#ifndef KIKS_GR_SIM_BRIDGE__EXPANDED_NODE_HPP_
#define KIKS_GR_SIM_BRIDGE__EXPANDED_NODE_HPP_

#include "kiks_gr_sim_bridge/node_expander.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"

namespace kiks::gr_sim_bridge
{

class ExpandedNode : public rclcpp::Node, public NodeExpander
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ExpandedNode)

  inline ExpandedNode(
    const std::string & name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ExpandedNode(name, "/", options)
  {}

  inline ExpandedNode(
    const std::string & name, const std::string & ns,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(name, ns, options),
    NodeExpander(*static_cast<rclcpp::Node *>(this))
  {}
};

}  // namespace kiks::gr_sim_bridge

#endif  // KIKS_GR_SIM_BRIDGE__EXPANDED_NODE_HPP_
