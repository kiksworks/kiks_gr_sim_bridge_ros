# [kiks_gr_sim_bridge_ros](../../../README.md)/[reference](../index.md)/class/sender_node

#include "kiks_gr_sim_bridge/sender_node.hpp"

kiks::gr_sim_bridge::SenderNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## Description
- This node sends packets written by subscriber nodes.
-  It has multiple [robot_receiver_node](robot_receiver_node.md)s and one [ball_receiver_node](ball_receiver_node.md) as members.

## public non-member function

#### static std::string default_name()
- Return default node name (="gr_sim_bridge")

## public member function

#### explicit SenderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name (="gr_sim_bridge_sender")

#### SenderNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument

#### SenderNode(const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments

#### explicit SenderNode(rclcpp::Node::SharedPtr node)
- Initialize with a node

## Related

### class
- [robot_receiver_node](robot_receiver_node.md)
- [ball_receiver_node](ball_receiver_node.md)

### node
- [sender](../node/sender.md)

###### &copy; 2023 KIKS