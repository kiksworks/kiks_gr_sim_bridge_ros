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
#include <memory>

#include "QNetworkInterface"
#include "QUdpSocket"

#include "grSim_Packet.pb.h"
#include "kiks_gr_sim_bridge/expanded_node.hpp"
#include "kiks_gr_sim_bridge/ball_subscriber_node.hpp"
#include "kiks_gr_sim_bridge/robot_subscriber_node.hpp"

namespace kiks::gr_sim_bridge
{

class SenderNode : public ExpandedNode
{
public:
  static auto default_name()
  {
    return "gr_sim_bridge";
  }

  explicit SenderNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : SenderNode(default_name(), "/", node_options)
  {
  }

  SenderNode(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : SenderNode(node_name, "/", node_options)
  {
  }

  SenderNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

private:
  using ParameterMsg = rcl_interfaces::msg::ParameterValue;
  using TwistMsg = RobotSubscriberNode::TwistMsg;
  using JointMsg = RobotSubscriberNode::JointMsg;
  using PoseMsg = RobotSubscriberNode::PoseMsg;
  using BallPoseMsg = BallSubscriberNode::PoseMsg;

  static inline std::vector<std::string> create_robot_names(
    const std::string & name_base,
    int count = 16);

  static inline void initialize_robot_cmd(int id, grSim_Robot_Command * robot_cmd);

  static inline void write_cmd_vel_to_packet(
    const TwistMsg & cmd_vel_msg, grSim_Packet * packet,
    int index = 0);

  static inline void write_cmd_spinner_to_packet(
    const JointMsg & cmd_spinner_msg,
    grSim_Packet * packet, int index = 0);

  static inline void write_cmd_flat_kick_to_packet(
    const JointMsg & cmd_flat_kick_msg,
    grSim_Packet * packet, int index = 0);

  static inline void write_cmd_chip_kick_to_packet(
    const JointMsg & cmd_chip_kick_msg,
    grSim_Packet * packet, int index = 0);

  static inline void write_robot_initialpose_to_packet(
    const PoseMsg & initialpose_msg,
    grSim_Packet * packet, int index = 0);

  static inline void write_ball_initialpose_to_packet(
    const BallPoseMsg & initialpose_msg,
    grSim_Packet * packet);

  inline void reset_subscriber_node_callbacks();

  inline void send_packet(const grSim_Packet & packet);

  // udp
  QUdpSocket udp_socket_;
  QHostAddress udp_gr_sim_address_;
  quint16 udp_port_;
  // robots & ball
  std::unordered_map<bool,
    std::unordered_map<std::uint32_t, RobotSubscriberNode>> robot_subscriber_nodes_map_;
  BallSubscriberNode ball_subscriber_node_;
  std::list<grSim_Packet> cmds_packets_, replacement_packets_;
  // callback handler
  rclcpp::TimerBase::SharedPtr sending_timer_, param_setting_timer_;
};

}  // namespace kiks::gr_sim_bridge

#endif  // KIKS_GR_SIM_BRIDGE__SENDER_NODE_HPP_
