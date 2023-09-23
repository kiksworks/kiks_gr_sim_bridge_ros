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
  this->add_parameter<double>(
    "sending_freq", 60.0, [this](const auto & param) {
      std::chrono::nanoseconds sending_duration(
        static_cast<int>(1000.0 * 1000.0 * 1000.0 /
        param.as_double()));
      sending_timer_ =
      node_->create_wall_timer(sending_duration, std::bind(&SenderNode::send, this));
    });
  // initialize robots by initializer func
  auto initialize_robots =
    [this](const std::string & team_str, TeamData & team_data, bool team_is_yellow) {
      // create default robot_strs
      std::vector<std::string> default_robot_strs;
      for (int i = 0; i < 16; ++i) {
        default_robot_strs.emplace_back(
          (i < 10) ?
          (team_str + "0" + std::to_string(i)) : (team_str + std::to_string(i)));
      }
      // set commnad team_color
      team_data.packet.mutable_commands()->set_isteamyellow(team_is_yellow);
      // Parameter of robot nodes
      // Default is {"${teamcolor}00, ${teamcolor}01 ... "${teamcolor}14", "${teamcolor}15"} for each team
      this->add_parameter<std::vector<std::string>>(
        team_str + "_robots", default_robot_strs,
        [this, &team_data, team_is_yellow](const auto & param) {
          set_robot_nodes(&team_data, param.get_value_message(), team_is_yellow);
        });
      // set robot nodes subscription
      team_data.robots_param_subscription = node_->create_subscription<ParameterMsg>(
        team_str + "_robots", this->get_static_qos(),
        [this, &team_data, team_is_yellow](ParameterMsg::ConstSharedPtr robots_param_msg) {
          set_robot_nodes(&team_data, *robots_param_msg, team_is_yellow);
        });
    };
  initialize_robots("yellow", yellow_, true);
  initialize_robots("blue", blue_, false);
  this->add_parameter<bool>(
    "ball.enable", true, [this](const auto & param) {
      if (param.as_bool()) {
        BallSubscriberNode::BallInfo ball_info;
        ball_info.replacement = replacement_packet_.mutable_replacement();
        ball_info.has_replacement = &has_replacement_;
        ball_subscriber_node_ = std::make_unique<BallSubscriberNode>(
          ball_info, node_->create_sub_node("ball"));
      } else {
        ball_subscriber_node_.reset();
      }
    });
  has_replacement_ = false;
}

void SenderNode::send()
{
  const auto now = node_->now();
  const double stamp = now.nanoseconds() * 0.001 * 0.001 * 0.001;
  auto send_packet = [now, stamp, this](TeamData & team_data) {
      team_data.packet.mutable_commands()->set_timestamp(stamp);
      for (auto & robot_subscriber_node : team_data.robot_subscriber_nodes) {
        robot_subscriber_node.update_validity(now);
      }
      const auto data_str = team_data.packet.SerializeAsString();
      udp_socket_.writeDatagram(data_str.data(), data_str.size(), udp_gr_sim_address_, udp_port_);
    };
  send_packet(yellow_);
  send_packet(blue_);
  if (has_replacement_) {
    const auto replacement_data_str = replacement_packet_.SerializeAsString();
    udp_socket_.writeDatagram(
      replacement_data_str.data(),
      replacement_data_str.size(), udp_gr_sim_address_, udp_port_);
    const auto replacement_ = replacement_packet_.mutable_replacement();
    replacement_->clear_ball();
    replacement_->clear_robots();
    has_replacement_ = false;
  }
}

void SenderNode::set_robot_nodes(
  TeamData * team_data, const ParameterMsg & param_msg,
  bool team_is_yellow)
{
  if (param_msg.type != 9) {
    RCLCPP_WARN(node_->get_logger(), "invalid parameter type(parameter should be string_array)");
    return;
  }
  team_data->robot_subscriber_nodes.clear();
  int robot_id = 0;
  const auto commands = team_data->packet.mutable_commands();
  const auto replacement = replacement_packet_.mutable_replacement();
  for (const auto & robot_str : param_msg.string_array_value) {
    if (robot_str == "") {
      continue;
    }
    const auto robot_command = commands->add_robot_commands();
    robot_command->set_id(robot_id);
    RobotSubscriberNode::RobotInfo robot_info;
    robot_info.command = robot_command;
    robot_info.replacement = replacement;
    robot_info.has_replacement = &has_replacement_;
    robot_info.team_is_yellow = team_is_yellow;
    robot_info.robot_id = robot_id;
    team_data->robot_subscriber_nodes.emplace_back(
      robot_info, node_->create_sub_node(robot_str));
    ++robot_id;
  }
}

}  // namespace kiks::gr_sim_bridge
