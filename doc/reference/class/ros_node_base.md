# [kiks_gr_sim_bridge_ros](../../../README.md)/[reference](../index.md)/class/ros_node_base

#include "kiks_gr_sim_bridge/ros_node_base.hpp"

kiks::gr_sim_bridge::RosNodeBase

#### inherited
- [robot_subscriber_node](robot_subscriber_node.md)
- [ball_subscriber_node](ball_subscriber_node.md)
- [sender_node](sender_node.md)

## Description
- This is the base class for ROS node.

## public member function

#### explicit RosNodeBase(rclcpp::Node::SharedPtr node)
- Initialize with a node

#### operator rclcpp::Node::SharedPtr()
- Returns a shared pointer to the nodes this class has

## protected member type

#### using SetParamFunc = std::function&lt;void (const rclcpp::Parameter &)&gt;
- Type of function called when parameter is set

## protected non-member function

#### static rclcpp::QoS get_dynamic_qos()
- Returns the qos to use for dynamic topics.

#### static rclcpp::QoS get_static_qos()
- Returns the qos to use for static topics.

## protected member function

#### template&lt;class T&gt; void add_parameter(std::string name, T default_value, SetParamFunc set_param_func)
- Register the parameters and the function called when the parameters are set in the node

## protected member variable

#### rclcpp::Node::SharedPtr node_
- The shared pointer of the nodes this class has

#### std::unordered_map&lt;std::string, SetParamFunc&gt; set_param_func_map_
- Map of functions to be called when parameters are set
- key : parameter name
- value : function called when parameter is set

###### &copy; 2023 KIKS