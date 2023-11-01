#include "kiks_gr_sim_bridge/node_expander.hpp"

namespace kiks::gr_sim_bridge
{

NodeExpander::NodeExpander(rclcpp::Node & node)
: node_(node),
  dynamic_qos_(rclcpp::QoS(1).durability_volatile()),
  dynamic_reliable_qos_(rclcpp::QoS(1).reliable().durability_volatile()),
  static_qos_(rclcpp::QoS(1).reliable().transient_local()),
#ifdef KIKS_ROS_DISTRO_DASHING
  params_setter_(node_.set_on_parameters_set_callback(
      [this](const auto & params) {return this->set_params(params);}))
#else
  params_setter_(
    node_.add_on_set_parameters_callback(
      [this](const auto & params) {return this->set_params(params);}))
#endif
{
  this->add_param(
    "dyanmic_qos.reliability", "best_effort", [this](const std::string & str) {
      if (str == "best_effort") {
        dynamic_qos_.best_effort();
      } else if (str == "reliable") {
        dynamic_qos_.reliable();
      } else {
        throw std::runtime_error("reliability must be \"best_effort\" or \"reliable\"");
      }
    });
}

inline NodeExpander::SetParamsResultMsg NodeExpander::set_params(
  const std::vector<rclcpp::Parameter> & params)
{
  SetParamsResultMsg set_params_result_msg;
  set_params_result_msg.successful = true;

  for (const auto & param : params) {
    const auto & name = param.get_name();
    const auto [begin, end] = param_setter_map_.equal_range(name);
    if (begin == end) {
      continue;
    }
    bool this_param_successful = true;
    auto itr = begin;
    do {
      const auto error_handle =
        [this, &set_params_result_msg, &this_param_successful](const std::string & reason) {
          RCLCPP_ERROR(node_.get_logger(), "set_param : %s", reason.c_str());
          if (set_params_result_msg.successful) {
            set_params_result_msg.successful = false;
          } else {
            set_params_result_msg.reason += " & ";
          }
          set_params_result_msg.reason += reason;
          this_param_successful = false;
        };
      try {
        itr->second(param);
      } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
        error_handle(name + " : ros error : " + e.what());
      } catch (const std::exception & e) {
        error_handle(name + " : std error :" + e.what());
      } catch (...) {
        error_handle(name + " : unknown error");
      }
      ++itr;
    } while(itr != end);

    if (this_param_successful) {
      RCLCPP_INFO(
        node_.get_logger(), "succes to set param : %s : %s", name.c_str(),
        param.value_to_string().c_str());
    }
  }
  return set_params_result_msg;
}

}  // namespace kiks::gr_sim_bridge
