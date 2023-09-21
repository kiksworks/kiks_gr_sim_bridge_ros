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


#include "kiks_gr_sim_bridge/ros_node_base.hpp"

#include <cstdint>
#include <vector>

namespace kiks::gr_sim_bridge
{

using namespace std::chrono_literals;

RosNodeBase::RosNodeBase(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
#ifdef KIKS_ROS_DISTRO_DASHING
  on_set_parameters_callback_handle_ = node_->set_on_parameters_set_callback(
    std::bind(&RosNodeBase::set_parameters, this, std::placeholders::_1));
#else
  on_set_parameters_callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&RosNodeBase::set_parameters, this, std::placeholders::_1));
#endif
}

rclcpp::QoS RosNodeBase::get_dynamic_qos()
{
  return rclcpp::QoS(1).best_effort().durability_volatile();
}

rclcpp::QoS RosNodeBase::get_static_qos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

template<typename T>
void RosNodeBase::add_parameter(std::string name, T default_value, SetParamFunc set_param_func)
{
  auto add_parameter_one_name = [this, set_param_func](const std::string & name, T default_value) {
      default_value = node_->has_parameter(name) ? node_->get_parameter(name).get_value<T>() :
        node_->declare_parameter<T>(name, default_value);
      set_param_func(rclcpp::Parameter(name, default_value));
      set_param_func_map_[name] = set_param_func;
      return default_value;
    };

  default_value = add_parameter_one_name(name, default_value);

  const auto & sub_namespace = node_->get_sub_namespace();
  if (sub_namespace.size()) {
    add_parameter_one_name(sub_namespace + "." + name, default_value);
  }
}

rcl_interfaces::msg::SetParametersResult RosNodeBase::set_parameters(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult set_param_result_msg;
  set_param_result_msg.successful = true;

  // エラー時に呼ばれる関数
  auto error_handle = [this, &set_param_result_msg](const std::string & error_reason) {
      RCLCPP_WARN(node_->get_logger(), "set_param : %s", error_reason.c_str());
      set_param_result_msg.successful = false;
      set_param_result_msg.reason = set_param_result_msg.reason.size() ?
        set_param_result_msg.reason + " & " + error_reason :
        error_reason;
    };
  // パラメータをセット
  for (const auto & parameter : parameters) {
    try {
      set_param_func_map_.at(parameter.get_name())(parameter);
    } catch (const std::out_of_range &) {
      // mapになかったら設定を行わない
      continue;
    } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
      // パラメータ設定時にROSのイベント例外が発生した場合
      error_handle(parameter.get_name() + " : ros error : " + e.what());
    } catch (const std::exception & e) {
      // パラメータ設定時にstd::exception例外が発生した場合
      error_handle(parameter.get_name() + " : std error :" + e.what());
    } catch (...) {
      // パラメータ設定時に設定していない例外が発生した場合
      error_handle(parameter.get_name() + " : unknown error");
    }
    // パラメータのセットに成功したら値を表示
    RCLCPP_INFO(
      node_->get_logger(), "succes to set param : %s : %s", parameter.get_name().c_str(),
      parameter.value_to_string().c_str());
  }
  return set_param_result_msg;
}

template void RosNodeBase::add_parameter<bool>(std::string, bool, SetParamFunc);
template void RosNodeBase::add_parameter<std::int64_t>(std::string, std::int64_t, SetParamFunc);
template void RosNodeBase::add_parameter<double>(std::string, double, SetParamFunc);
template void RosNodeBase::add_parameter<std::string>(std::string, std::string, SetParamFunc);
template void RosNodeBase::add_parameter<std::vector<std::uint8_t>>(
  std::string, std::vector<std::uint8_t>, SetParamFunc);
template void RosNodeBase::add_parameter<std::vector<bool>>(
  std::string, std::vector<bool>, SetParamFunc);
template void RosNodeBase::add_parameter<std::vector<std::int64_t>>(
  std::string, std::vector<std::int64_t>, SetParamFunc);
template void RosNodeBase::add_parameter<std::vector<double>>(
  std::string, std::vector<double>, SetParamFunc);
template void RosNodeBase::add_parameter<std::vector<std::string>>(
  std::string, std::vector<std::string>, SetParamFunc);

}  // namespace kiks::gr_sim_bridge
