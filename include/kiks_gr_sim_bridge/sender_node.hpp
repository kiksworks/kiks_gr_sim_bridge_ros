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


#ifndef KIKS_GR_SIM_BRIDGE__SENDER_NODE_HPP_
#define KIKS_GR_SIM_BRIDGE__SENDER_NODE_HPP_

#include <string>
#include <list>

#include "QNetworkInterface"
#include "QUdpSocket"

#include "kiks_gr_sim_bridge/ros_node_base.hpp"
#include "kiks_gr_sim_bridge/robot_subscriber_node.hpp"
#include "grSim_Packet.pb.h"

namespace kiks::gr_sim_bridge
{

class SenderNode : public RosNodeBase
{
public:
  static std::string default_name();

  explicit SenderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  SenderNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  SenderNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit SenderNode(rclcpp::Node::SharedPtr node);

private:
  struct TeamData
  {
    grSim_Packet packet;
    std::list<RobotSubscriberNode> robot_subscriber_nodes;
  };

  void send();

  QUdpSocket udp_socket_;
  QHostAddress udp_gr_sim_address_;
  quint16 udp_port_;
  TeamData yellow_, blue_;
  rclcpp::TimerBase::SharedPtr sending_timer_;
};

}  // namespace kiks::gr_sim_bridge

#endif  // KIKS_GR_SIM_BRIDGE__SENDER_NODE_HPP_
