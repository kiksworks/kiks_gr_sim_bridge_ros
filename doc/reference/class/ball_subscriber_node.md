# [kiks_gr_sim_bridge_ros](../../../README.md)/[reference](../index.md)/class/ball_subscriber_node

#include "kiks_gr_sim_bridge/ball_subscriber_node.hpp"

kiks::gr_sim_bridge::BallSubscriberNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## Description
- This node writes the data of the subscribed ball to the given packet
- Node alone does not work.

## public type
#### BallInfo

## public non-member function

#### static std::string default_name()
- Return default node name (="gr_sim_bridge_ball_subscriber")

## public member function

#### explicit BallSubscriberNode(const BallInfo robot_info, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name (="gr_sim_bridge_ball_subscriber")

#### BallSubscriberNode(const BallInfo robot_info, const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument

#### BallSubscriberNode(const BallInfo robot_info, const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments

#### explicit BallSubscriberNode(const BallInfo robot_info, rclcpp::Node::SharedPtr node)
- Initialize with a node

## Related

### class
- [sender_node](sender_node.md)

### node
- [ball_subscriber](../node/ball_subscriber.md)

###### &copy; 2023 KIKS