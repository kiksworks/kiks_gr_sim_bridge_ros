# [kiks_gr_sim_bridge_ros](../../../README.md)/[reference](../index.md)/class/robot_subscriber_node

#include "kiks_gr_sim_bridge/robot_subscriber_node.hpp"

kiks::gr_sim_bridge::RobotSubscriberNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## Description
- This node writes the data of the subscribed robot to the given packet.
- Node alone does not work.

## public type
#### RobotInfo

## public non-member function

#### static std::string default_name()
- Return default node name (="gr_sim_bridge_robot_subscriber")

## public member function

#### explicit RobotSubscriberNode(const RobotInfo robot_info, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name (="gr_sim_bridge_robot_subscriber")

#### RobotSubscriberNode(const RobotInfo robot_info, const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument

#### RobotSubscriberNode(const RobotInfo robot_info, const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments

#### explicit RobotSubscriberNode(const RobotInfo robot_info, rclcpp::Node::SharedPtr node)
- Initialize with a node

## Related

### class
- [sender_node](sender_node.md)

### node
- [robot_subscriber](../node/robot_subscriber.md)

###### &copy; 2023 KIKS