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
  ball_subscriber_node_(this->create_sub_node("ball"))
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
    "rate", 0.0, [this](double rate) {
      sending_timer_ = (rate == 0) ? rclcpp::TimerBase::SharedPtr() :
      this->create_wall_timer(
        std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / rate)), [this]() {
          for (auto & cmds_packet : cmds_packets_) {
            cmds_packet.mutable_commands()->set_timestamp(1e-9 * this->now().nanoseconds());
            this->send_packet(cmds_packet);
            for (int i = 0; i < cmds_packet.mutable_commands()->robot_commands_size(); ++i) {
              write_cmd_flat_kick_to_packet(JointMsg(), &cmds_packet, i);
            }
          }
          for (auto & replacement_packet : replacement_packets_) {
            this->send_packet(replacement_packet);
            replacement_packet.clear_replacement();
          }
        });
      this->reset_subscriber_node_callbacks();
    });
  // Parameter of robots[hz]
  robot_subscriber_nodes_map_[true];
  robot_subscriber_nodes_map_[false];
  auto feature_robots_param_setter = [this] {
      for (auto & [isteamyellow, robot_subscriber_nodes] : robot_subscriber_nodes_map_) {
        std::string color = isteamyellow ? "yellow" : "blue";
        this->add_param(
          color + "_robots", this->create_robot_names(color),
          [this, &robot_subscriber_nodes](const std::vector<std::string> & names) {
            robot_subscriber_nodes.clear();
            int id = 0;
            for (const auto & name : names) {
              if (name == "/") {
                robot_subscriber_nodes.emplace(id, this->shared_from_this());
              } else if (name != "") {
                robot_subscriber_nodes.emplace(id, this->create_sub_node(name));
              }
              ++id;
            }
          });
        this->reset_subscriber_node_callbacks();
      }
      param_setting_timer_.reset();
    };
  param_setting_timer_ = this->create_wall_timer(
    std::chrono::nanoseconds::zero(), feature_robots_param_setter);
}

std::vector<std::string> SenderNode::create_robot_names(const std::string & name_base, int count)
{
  std::vector<std::string> names;
  for (int i = 0; i < count; ++i) {
    names.emplace_back(
      i <
      10 ? name_base + "0" + std::to_string(i) : name_base + std::to_string(i));
  }
  return names;
}

void SenderNode::initialize_robot_cmd(int id, grSim_Robot_Command * robot_cmd)
{
  robot_cmd->set_id(id);
  robot_cmd->set_veltangent(0);
  robot_cmd->set_velnormal(0);
  robot_cmd->set_velangular(0);
  robot_cmd->set_spinner(false);
  robot_cmd->set_kickspeedx(0);
  robot_cmd->set_kickspeedz(0);
  robot_cmd->set_wheelsspeed(false);
}

void SenderNode::write_cmd_vel_to_packet(
  const TwistMsg & cmd_vel_msg, grSim_Packet * packet,
  int index)
{
  auto robot_cmd = packet->mutable_commands()->mutable_robot_commands(index);
  robot_cmd->set_veltangent(cmd_vel_msg.linear.x);
  robot_cmd->set_velnormal(cmd_vel_msg.linear.y);
  robot_cmd->set_velangular(cmd_vel_msg.angular.z);
}

void SenderNode::write_cmd_spinner_to_packet(
  const JointMsg & cmd_spinner_msg,
  grSim_Packet * packet, int index)
{
  packet->mutable_commands()->mutable_robot_commands(index)->set_spinner(
    cmd_spinner_msg.velocity > 0);
}

void SenderNode::write_cmd_flat_kick_to_packet(
  const JointMsg & cmd_flat_kick_msg,
  grSim_Packet * packet, int index)
{
  auto robot_cmd = packet->mutable_commands()->mutable_robot_commands(index);
  robot_cmd->set_kickspeedx(cmd_flat_kick_msg.velocity);
  robot_cmd->set_kickspeedz(0);
}

void SenderNode::write_cmd_chip_kick_to_packet(
  const JointMsg & cmd_chip_kick_msg,
  grSim_Packet * packet, int index)
{
  auto robot_cmd = packet->mutable_commands()->mutable_robot_commands(index);
  constexpr auto kick_angle = 60 / 180 * std::acos(-1.0);
  robot_cmd->set_kickspeedx(cmd_chip_kick_msg.velocity * std::cos(kick_angle));
  robot_cmd->set_kickspeedz(cmd_chip_kick_msg.velocity * std::sin(kick_angle));
}

void SenderNode::write_robot_initialpose_to_packet(
  const PoseMsg & initialpose_msg,
  grSim_Packet * packet, int index)
{
  auto robot_replacement = packet->mutable_replacement()->mutable_robots(index);
  robot_replacement->set_x(initialpose_msg.pose.pose.position.x);
  robot_replacement->set_y(initialpose_msg.pose.pose.position.y);
  robot_replacement->set_dir(
    std::atan2(
      initialpose_msg.pose.pose.orientation.z,
      initialpose_msg.pose.pose.orientation.w) * (2 * 180.0 / std::acos(-1.0)));
}

void SenderNode::write_ball_initialpose_to_packet(
  const BallPoseMsg & initialpose_msg,
  grSim_Packet * packet)
{
  auto ball_replacement = packet->mutable_replacement()->mutable_ball();
  ball_replacement->set_x(initialpose_msg.pose.pose.position.x);
  ball_replacement->set_y(initialpose_msg.pose.pose.position.y);
}

inline void SenderNode::reset_subscriber_node_callbacks()
{
  cmds_packets_.clear();
  replacement_packets_.clear();
  if (sending_timer_) {
    // set callback to sync
    auto & replacement_packet = replacement_packets_.emplace_back();
    for (auto & [isteamyellow, robot_subscriber_nodes] : robot_subscriber_nodes_map_) {
      auto & packet = cmds_packets_.emplace_back();
      auto cmd = packet.mutable_commands();
      cmd->set_isteamyellow(isteamyellow);
      for (auto & [id, robot_subscriber_node] : robot_subscriber_nodes) {
        auto index = cmd->robot_commands_size();
        auto robot_command = cmd->add_robot_commands();
        initialize_robot_cmd(id, robot_command);
        robot_subscriber_node.set_cmd_vel_call_back(
          [this, &packet, index](TwistMsg::ConstSharedPtr cmd_vel_msg) {
            this->write_cmd_vel_to_packet(*cmd_vel_msg, &packet, index);
          }, [this, &packet, index] {
            this->write_cmd_vel_to_packet(TwistMsg(), &packet, index);
          });
        robot_subscriber_node.set_cmd_spinner_callback(
          [this, &packet, index](JointMsg::ConstSharedPtr cmd_spinner_msg) {
            this->write_cmd_spinner_to_packet(*cmd_spinner_msg, &packet, index);
          });
        robot_subscriber_node.set_cmd_flat_kick_callback(
          [this, &packet, index](JointMsg::ConstSharedPtr cmd_flat_kick_msg) {
            this->write_cmd_flat_kick_to_packet(*cmd_flat_kick_msg, &packet, index);
          });
        robot_subscriber_node.set_cmd_chip_kick_callback(
          [this, &packet, index](JointMsg::ConstSharedPtr cmd_chip_kick_msg) {
            this->write_cmd_spinner_to_packet(*cmd_chip_kick_msg, &packet, index);
          });
        robot_subscriber_node.set_initialpose_callback(
          [this, &replacement_packet](PoseMsg::ConstSharedPtr initialpose_msg) {
            auto replacement = replacement_packet.mutable_replacement();
            auto index = replacement->robots_size();
            replacement->add_robots();
            this->write_robot_initialpose_to_packet(*initialpose_msg, &replacement_packet, index);
          });
      }
    }
    ball_subscriber_node_.set_initialpose_callback(
      [this, &replacement_packet](BallPoseMsg::ConstSharedPtr initialpose_msg) {
        this->write_ball_initialpose_to_packet(*initialpose_msg, &replacement_packet);
      });
  } else {
    // set callback to async
    for (auto & [isteamyellow, robot_subscriber_nodes] : robot_subscriber_nodes_map_) {
      for (auto & [id, robot_subscriber_node] : robot_subscriber_nodes) {
        auto & packet = cmds_packets_.emplace_back();
        auto cmd = packet.mutable_commands();
        cmd->set_isteamyellow(isteamyellow);
        auto robot_command = cmd->add_robot_commands();
        initialize_robot_cmd(id, robot_command);
        robot_subscriber_node.set_cmd_vel_call_back(
          [this, &packet](TwistMsg::ConstSharedPtr cmd_vel_msg) {
            packet.mutable_commands()->set_timestamp(1e-9 * this->now().nanoseconds());
            this->write_cmd_vel_to_packet(*cmd_vel_msg, &packet);
            this->send_packet(packet);
          }, [this, &packet] {
            packet.mutable_commands()->set_timestamp(1e-9 * this->now().nanoseconds());
            this->write_cmd_vel_to_packet(TwistMsg(), &packet);
            this->send_packet(packet);
          });
        robot_subscriber_node.set_cmd_spinner_callback(
          [this, &packet](JointMsg::ConstSharedPtr cmd_spinner_msg) {
            packet.mutable_commands()->set_timestamp(1e-9 * this->now().nanoseconds());
            this->write_cmd_spinner_to_packet(*cmd_spinner_msg, &packet);
            this->send_packet(packet);
          });
        robot_subscriber_node.set_cmd_flat_kick_callback(
          [this, &packet](JointMsg::ConstSharedPtr cmd_flat_kick_msg) {
            packet.mutable_commands()->set_timestamp(1e-9 * this->now().nanoseconds());
            this->write_cmd_flat_kick_to_packet(*cmd_flat_kick_msg, &packet);
            this->send_packet(packet);
          });
        robot_subscriber_node.set_cmd_chip_kick_callback(
          [this, &packet](JointMsg::ConstSharedPtr cmd_chip_kick_msg) {
            packet.mutable_commands()->set_timestamp(1e-9 * this->now().nanoseconds());
            this->write_cmd_spinner_to_packet(*cmd_chip_kick_msg, &packet);
            this->send_packet(packet);
          });
        auto & replacement_packet = replacement_packets_.emplace_back();
        robot_subscriber_node.set_initialpose_callback(
          [this, &replacement_packet, id](PoseMsg::ConstSharedPtr initialpose_msg) {
            auto replacement = replacement_packet.mutable_replacement();
            auto index = replacement->robots_size();
            auto robot_replacement = replacement->add_robots();
            robot_replacement->set_id(id);
            this->write_robot_initialpose_to_packet(*initialpose_msg, &replacement_packet, index);
            this->send_packet(replacement_packet);
          });
      }
    }
    auto & replacement_packet = replacement_packets_.emplace_back();
    ball_subscriber_node_.set_initialpose_callback(
      [this, &replacement_packet](BallPoseMsg::ConstSharedPtr initialpose_msg) {
        this->write_ball_initialpose_to_packet(*initialpose_msg, &replacement_packet);
        this->send_packet(replacement_packet);
      });
  }
}

inline void SenderNode::send_packet(const grSim_Packet & packet)
{
  auto packet_str = packet.SerializeAsString();
  udp_socket_.writeDatagram(packet_str.data(), packet_str.size(), udp_gr_sim_address_, udp_port_);
}

}  // namespace kiks::gr_sim_bridge
