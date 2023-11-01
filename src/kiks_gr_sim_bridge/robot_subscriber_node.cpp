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
  this->add_param("timeout_duration", 0.1, [this] (double duration) {
      cmd_vel_timeout_duration_ = std::chrono::nanoseconds(std::int64_t(1e9 * duration));
    });
}

// void RobotSubscriberNode::update_validity(const rclcpp::Time & now)
// {
//   if (now > vel_valid_time_) {
//     robot_info_.command->set_veltangent(0);
//     robot_info_.command->set_velnormal(0);
//     robot_info_.command->set_velangular(0);
//   }
//   if (now > kick_valid_time_) {
//     robot_info_.command->set_kickspeedx(0);
//     robot_info_.command->set_kickspeedz(0);
//   }
// }

// void RobotSubscriberNode::subscribe_cmd_vel(TwistMsg::ConstSharedPtr cmd_vel_msg)
// {
//   vel_valid_time_ = (*this)->now() + vel_valid_duration_;
//   robot_info_.command->set_veltangent(cmd_vel_msg->linear.x);
//   robot_info_.command->set_velnormal(cmd_vel_msg->linear.y);
//   robot_info_.command->set_velangular(cmd_vel_msg->angular.z);
// }

// void RobotSubscriberNode::subscribe_cmd_flat_kick(JointMsg::ConstSharedPtr cmd_flat_kick_msg)
// {
//   kick_valid_time_ = (*this)->now() + kick_valid_duration_;
//   robot_info_.command->set_kickspeedx(cmd_flat_kick_msg->velocity);
//   robot_info_.command->set_kickspeedz(0);
// }

// void RobotSubscriberNode::subscribe_cmd_chip_kick(JointMsg::ConstSharedPtr cmd_chip_kick_msg)
// {
//   kick_valid_time_ = (*this)->now() + kick_valid_duration_;
//   robot_info_.command->set_kickspeedx(cmd_chip_kick_msg->velocity * chip_kick_coef_x_);
//   robot_info_.command->set_kickspeedz(cmd_chip_kick_msg->velocity * chip_kick_coef_z_);
// }


// void RobotSubscriberNode::subscribe_cmd_spinner(JointMsg::ConstSharedPtr cmd_spinner_msg)
// {
//   robot_info_.command->set_spinner(cmd_spinner_msg->velocity > 0);
// }

// void RobotSubscriberNode::subscribe_initialpose(PoseMsg::ConstSharedPtr initialpose_msg)
// {
//   auto robot = robot_info_.replacement->add_robots();
//   const auto & pose = initialpose_msg->pose.pose;
//   robot->set_x(pose.position.x);
//   robot->set_y(pose.position.y);
//   robot->set_dir(
//     std::atan2(
//       pose.orientation.z,
//       pose.orientation.w) * (2 * 180.0 / std::acos(-1.0)));
//   robot->set_yellowteam(robot_info_.team_is_yellow);
//   robot->set_id(robot_info_.robot_id);
//   *robot_info_.has_replacement = true;
// }

}  // namespace kiks::gr_sim_bridge
