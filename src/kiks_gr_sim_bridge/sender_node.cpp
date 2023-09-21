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


#include "kiks_gr_sim_bridge/sender_node.hpp"

#include <chrono>

#include "grSim_Packet.pb.h"

namespace kiks::gr_sim_bridge
{

SenderNode::SenderNode(const rclcpp::NodeOptions & options)
: SenderNode(std::make_shared<rclcpp::Node>("gr_sim_bridge", options))
{
}

SenderNode::SenderNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: SenderNode(std::make_shared<rclcpp::Node>(node_name, options))
{
}

SenderNode::SenderNode(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: SenderNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

SenderNode::SenderNode(rclcpp::Node::SharedPtr node)
: RosNodeBase(std::move(node))
{
  // Parameter of ssl-vision udp port
  this->add_parameter<std::int64_t>(
    "udp.port", 20011, [this](const auto & param) {
      udp_port_ = param.as_int();
    });
  // Parameter of ssl-vision udp multicast ip
  this->add_parameter<std::string>(
    "udp.gr_sim_address", "127.0.0.1", [this](const auto & param) {
      udp_gr_sim_address_ = QHostAddress(param.as_string().c_str());
    });
  // Parameter of sending_timer update frequency[hz]
  this->add_parameter<double>("sending_freq", 60.0, [this](const auto & param){
    std::chrono::nanoseconds sending_duration(static_cast<int>(1000.0 * 1000.0 * 1000.0 / param.as_double()));
    sending_timer_ =
        node_->create_wall_timer(sending_duration, std::bind(&SenderNode::send, this));
  });
}

void SenderNode::send()
{
  grSim_Packet packet;
  auto data_str = packet.SerializeAsString();
  udp_socket_.writeDatagram(data_str.data(), data_str.size(), udp_gr_sim_address_, udp_port_);
}

}  // namespace kiks::gr_sim_bridge
