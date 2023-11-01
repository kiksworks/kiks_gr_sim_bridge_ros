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


#ifndef KIKS_GR_SIM_BRIDGE__NODE_EXPANDER_HPP_
#define KIKS_GR_SIM_BRIDGE__NODE_EXPANDER_HPP_

#include <deque>
#include <functional>
#include <string>
#include <type_traits>
#include <unordered_map>

#include "rclcpp/node.hpp"

namespace kiks::gr_sim_bridge
{

class NodeExpander
{
public:
  NodeExpander(rclcpp::Node & node);

protected:
  using SetParamFunc = std::function<void (const rclcpp::Parameter &)>;
  using SetParamsResultMsg = rcl_interfaces::msg::SetParametersResult;
  template<class T>
  using ParamValType = decltype(rclcpp::Parameter().get_value<T>());
  template<class Func, class FuncBase, class Ret = void>
  using IfEnableByFuncConvertible = std::enable_if<std::is_convertible<Func,
      std::function<FuncBase>>::value, Ret>;

  inline const rclcpp::QoS &
  get_dynamic_qos() const noexcept
  {
    return dynamic_qos_;
  }

  inline const rclcpp::QoS &
  get_dynamic_reliable_qos() const noexcept
  {
    return dynamic_reliable_qos_;
  }

  inline const rclcpp::QoS &
  get_static_qos() const noexcept
  {
    return static_qos_;
  }

  template<typename T, class Func>
  inline typename std::enable_if<std::is_same<T, ParamValType<T>>::value,
    IfEnableByFuncConvertible<Func, void(rclcpp::Parameter)>>::type::type
  add_param(std::string && name, const T & default_value, const Func & param_setter)
  {
    auto value = default_value;
    for (const auto & n : get_param_names(std::move(name))) {
      value = node_.has_parameter(n) ?
        node_.get_parameter(n).get_value<T>() :
        node_.declare_parameter<T>(n, value);
      param_setter(rclcpp::Parameter(n, value));
      param_setter_map_.emplace(n, param_setter);
    }
  }

  template<typename T, class Func>
  inline typename std::enable_if<!std::is_same<T, ParamValType<T>>::value,
    IfEnableByFuncConvertible<Func, void(rclcpp::Parameter)>>::type::type
  add_param(std::string && name, const T & default_value, const Func & param_setter)
  {
    add_param<ParamValType<T>>(
      std::move(name), default_value, param_setter);
  }

  template<typename T, class Func>
  inline typename IfEnableByFuncConvertible<Func, void(ParamValType<T>)>::type
  add_param(std::string && name, const T & default_value, const Func & param_setter)
  {
    add_param<ParamValType<T>>(
      std::move(name), default_value,
      [param_setter](const rclcpp::Parameter & param) {
        return param_setter(param.get_value<T>());
      });
  }

  template<typename T, class Func>
  inline typename IfEnableByFuncConvertible<Func, void(void)>::type
  add_param(std::string && name, const T & default_value, const Func & param_setter)
  {
    add_param<ParamValType<T>>(
      std::move(name), default_value,
      [param_setter](const rclcpp::Parameter &) {
        return param_setter();
      });
  }

  template<typename T, class Parent, class... FuncArgs>
  inline void add_param(std::string && name, const T & default_value, void (Parent::*param_setter)(FuncArgs...))
  {
    add_param<ParamValType<T>>(
      std::move(name), default_value,
      [this, param_setter](const FuncArgs & ... args) {
        return (static_cast<Parent*>(this)->*param_setter)(args...);
      });
  }

  std::unordered_multimap<std::string, SetParamFunc> param_setter_map_;

private:
  inline SetParamsResultMsg set_params(
    const std::vector<rclcpp::Parameter> & params);

  inline std::deque<std::string> get_param_names(std::string name) const
  {
    std::deque<std::string> names;
    names.emplace_back(std::move(name));
    const auto & sub_ns = node_.get_sub_namespace();
    std::string::size_type pos_begin = 0, pos_end;
    std::string param_ns;
    do {
      pos_end = sub_ns.find_first_of('/', pos_begin);
      const auto len = pos_end - pos_begin;
      auto parsed_ns = sub_ns.substr(pos_begin, len);
      if (parsed_ns == "") {
        continue;
      }
      param_ns += parsed_ns + ".";
      names.emplace_back(param_ns + names.front());
      pos_begin = pos_end + 1;
    } while (pos_end != std::string::npos);
    return names;
  }

  rclcpp::Node & node_;
  rclcpp::QoS dynamic_qos_, dynamic_reliable_qos_, static_qos_;

#ifdef KIKS_ROS_DISTRO_DASHING
  const rclcpp::Node::OnParametersSetCallbackType params_setter_;
#else
  const rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
    params_setter_;
#endif
};

}  // namespace kiks::gr_sim_bridge

#endif  // KIKS_GR_SIM_BRIDGE__NODE_EXPANDER_HPP_
