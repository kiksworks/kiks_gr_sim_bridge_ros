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
  // Parameter of each team robots
  // Default is {"${teamcolor}00, ${teamcolor}01 ... "${teamcolor}14", "${teamcolor}15"} for each team
  auto create_robots_str = [this](const std::string & base_name) {
      std::vector<std::string> namespaces;
      for (int i = 0; i < 16; ++i) {
        const auto ns =
          (i < 10) ? (base_name + "0" + std::to_string(i)) : (base_name + std::to_string(i));
        namespaces.push_back(ns);
      }
      return namespaces;
    };
  auto set_robots = [this](TeamData& data, const std::vector<std::string>& str_array, bool team_is_yellow) {
      data.robot_subscriber_nodes.clear();
      const auto commands = data.packet.mutable_commands();
      commands->set_isteamyellow(team_is_yellow);
      const auto replacement = data.packet.mutable_replacement();
      commands->clear_robot_commands();
      for (std::uint32_t i = 0; i < str_array.size(); ++i) {
        const auto & str = str_array[i];
        if (str == "") {
          continue;
        }
        const auto robot_command = commands->add_robot_commands();
        robot_command->set_id(i);
        data.robot_subscriber_nodes.emplace_back(robot_command, replacement, team_is_yellow, node_->create_sub_node(str));
      }
    };
  this->add_parameter<std::vector<std::string>>(
    "yellow_robots", create_robots_str("yellow"), [this, set_robots](const auto & param) {
      set_robots(yellow_, param.as_string_array(), true);
    });
  this->add_parameter<std::vector<std::string>>(
    "blue_robots", create_robots_str("blue"), [this, set_robots](const auto & param) {
      set_robots(blue_, param.as_string_array(), false);
    });
}

void SenderNode::send()
{
  auto send_packet = [this](const auto& packet) {
      const auto data_str = packet.SerializeAsString();
      udp_socket_.writeDatagram(data_str.data(), data_str.size(), udp_gr_sim_address_, udp_port_);
    };
  send_packet(yellow_.packet);
  send_packet(blue_.packet);
}

}  // namespace kiks::gr_sim_bridge
