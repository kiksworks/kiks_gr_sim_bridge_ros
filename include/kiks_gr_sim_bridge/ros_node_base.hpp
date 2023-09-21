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


#ifndef KIKS_GR_SIM_BRIDGE__ROS_NODE_BASE_HPP_
#define KIKS_GR_SIM_BRIDGE__ROS_NODE_BASE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/node.hpp"

namespace kiks::gr_sim_bridge
{

class RosNodeBase
{
public:
  explicit RosNodeBase(rclcpp::Node::SharedPtr node);

  inline operator rclcpp::Node::SharedPtr() const noexcept {return node_;}

protected:
  using SetParamFunc = std::function<void (const rclcpp::Parameter &)>;

  static rclcpp::QoS get_dynamic_qos();

  static rclcpp::QoS get_static_qos();

  template<typename T>
  void add_parameter(std::string name, T default_value, SetParamFunc set_param_func);

  template<typename T, typename MemberFunc, typename ClassPtr>
  void add_parameter(std::string name, T default_value, MemberFunc member_func, ClassPtr class_ptr)
  {
    create_parameter(
      std::move(name), default_value, std::bind(member_func, class_ptr, std::placeholders::_1));
  }

  rclcpp::Node::SharedPtr node_;

  std::unordered_map<std::string, SetParamFunc> set_param_func_map_;

private:
  rcl_interfaces::msg::SetParametersResult set_parameters(
    std::vector<rclcpp::Parameter> parameters);

#ifdef KIKS_ROS_DISTRO_DASHING
  rclcpp::Node::OnParametersSetCallbackType on_set_parameters_callback_handle_;
#else
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;
#endif
};

}  // namespace kiks::gr_sim_bridge

#endif  // KIKS_GR_SIM_BRIDGE__ROS_NODE_BASE_HPP_
