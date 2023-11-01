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

SenderNode::SenderNode(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: ExpandedNode(node_name, node_namespace, options),
  ball_subscriber_node_(BallSubscriberNode::BallInfo(), this->create_sub_node("ball"))
{
  // Parameter of ssl-vision udp port
  this->add_param<std::int64_t>(
    "udp.port", 20011, [this](std::int64_t port) {
      udp_port_ = port;
    });
  // Parameter of ssl-vision udp multicast ip
  this->add_param<std::string>(
    "udp.gr_sim_address", "127.0.0.1", [this](const std::string & address) {
      udp_gr_sim_address_ = QHostAddress(address.c_str());
    });
  // Parameter of sending_timer update frequency[hz]
  this->add_param<double>(
    "rate", 0.0, &SenderNode::set_rate);
  // initialize robots by initializer func
  // auto initialize_robots =
  //   [this](const std::string & team_str, bool team_is_yellow) {
  //     // create default robot_strs
  //     std::vector<std::string> default_robot_strs;
  //     for (int i = 0; i < 16; ++i) {
  //       default_robot_strs.emplace_back(
  //         (i < 10) ?
  //         (team_str + "0" + std::to_string(i)) : (team_str + std::to_string(i)));
  //     }
  //     // set commnad team_color
  //     team_data.packet.mutable_commands()->set_isteamyellow(team_is_yellow);
  //     // Parameter of robot nodes
  //     // Default is
  //     // {"${teamcolor}00", "${teamcolor}01" ... "${teamcolor}14", "${teamcolor}15"}
  //     // for each team
  //     this->add_param<std::vector<std::string>>(
  //       team_str + "_robots", default_robot_strs,
  //       [this, &team_data, team_is_yellow](const rclcpp::Parameter & param) {
  //         set_robot_nodes(&team_data, param.get_value_message(), team_is_yellow);
  //       });
  //     // set robot nodes subscription
  //     team_data.robots_param_subscription = this->create_subscription<ParameterMsg>(
  //       team_str + "_robots", this->get_static_qos(),
  //       [this, &team_data, team_is_yellow](ParameterMsg::ConstSharedPtr robots_param_msg) {
  //         set_robot_nodes(&team_data, *robots_param_msg, team_is_yellow);
  //       });
  //   };
  // initialize_robots("yellow", yellow_, true);
  // initialize_robots("blue", blue_, false);
}


void SenderNode::write_cmd_vel_to_packet(const TwistMsg & cmd_vel_msg, grSim_Packet * packet, int index)
{
  auto robot_cmd = packet->mutable_commands()->mutable_robot_commands(index);
  robot_cmd->set_veltangent(cmd_vel_msg.linear.x);
  robot_cmd->set_velnormal(cmd_vel_msg.linear.y);
  robot_cmd->set_velangular(cmd_vel_msg.angular.z);
}

void SenderNode::write_robot_initialpose_to_packet(const PoseMsg & initialpose_msg, grSim_Packet * packet, int index)
{
  auto robot_replacement = packet->mutable_replacement()->mutable_robots(index);
  robot_replacement->set_x(initialpose_msg.pose.pose.position.x);
  robot_replacement->set_y(initialpose_msg.pose.pose.position.y);
  robot_replacement->set_dir(
    std::atan2(
      initialpose_msg.pose.pose.orientation.z,
      initialpose_msg.pose.pose.orientation.w) * (2 * 180.0 / std::acos(-1.0)));
}

void SenderNode::write_ball_initialpose_to_packet(const PoseMsg & initialpose_msg, grSim_Packet * packet)
{
  auto ball_replacement = packet->mutable_replacement()->mutable_ball();
  ball_replacement->set_x(initialpose_msg.pose.pose.position.x);
  ball_replacement->set_y(initialpose_msg.pose.pose.position.y);
}

void SenderNode::set_rate(double rate)
{
  sending_timer_ = (rate == 0) ? rclcpp::TimerBase::SharedPtr() :
    this->create_wall_timer(std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / rate)), [this] () {
        for(const auto & cmds_packet : cmds_packets_) {
          this->send_packet(cmds_packet);
        }
        for(auto & replacement_packet : replacement_packets_) {
          this->send_packet(replacement_packet);
          replacement_packet.clear_replacement();
        }
      });

  this->reset_subscriber_node_callbacks();
}

inline void SenderNode::reset_subscriber_node_callbacks()
{
  cmds_packets_.clear();
  replacement_packets_.clear();
  if (sending_timer_) {
    for (auto & [isteamyellow, robot_subscriber_nodes] : robot_subscriber_nodes_map_) {
      auto & packet = cmds_packets_.emplace_back();
      auto cmd = packet.mutable_commands();
      cmd->set_isteamyellow(isteamyellow);
      for (auto & [id, robot_subscriber_node] : robot_subscriber_nodes) {
        auto index = cmd->robot_commands_size();
        auto robot_command = cmd->add_robot_commands();
        robot_command->set_id(id);
        robot_subscriber_node.set_cmd_vel_call_back([this, &packet, index](TwistMsg::ConstSharedPtr cmd_vel_msg) {
            this->write_cmd_vel_to_packet(*cmd_vel_msg, &packet, index);
          }, [this, &packet, index] {
            this->write_cmd_vel_to_packet(TwistMsg(), &packet, index);
          });
      }
    }
  }
  else {
    for (auto & [isteamyellow, robot_subscriber_nodes] : robot_subscriber_nodes_map_) {
      for (auto & [id, robot_subscriber_node] : robot_subscriber_nodes) {
        auto & packet = cmds_packets_.emplace_back();
        auto cmd = packet.mutable_commands();
        cmd->set_isteamyellow(isteamyellow);
        auto robot_command = cmd->add_robot_commands();
        robot_command->set_id(id);
        robot_subscriber_node.set_cmd_vel_call_back([this, &packet](TwistMsg::ConstSharedPtr cmd_vel_msg) {
            this->write_cmd_vel_to_packet(*cmd_vel_msg, &packet);
            this->send_packet(packet);
          }, [this, packet] {
            this->send_packet(packet);
          });
      }
    }
  }
}

inline void SenderNode::send_packet(const grSim_Packet & packet)
{
  auto packet_str = packet.SerializeAsString();
  udp_socket_.writeDatagram(packet_str.data(), packet_str.size(), udp_gr_sim_address_, udp_port_);
}

}  // namespace kiks::gr_sim_bridge
