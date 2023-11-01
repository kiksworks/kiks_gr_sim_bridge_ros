# [kiks_gr_sim_bridge_ros](../../README.md)/jp

現在のバージョン(v0.1.0)の日本語のリファレンスです。

## [executor](./jp/executor.md)

| 名前 | 説明 |
|-|-|
| [gr_sim_bridge](./executor/gr_sim_bridge.md) | ROS2トピックをUDPパケットにブリッジします。 |

## [node](./jp/node.md)

| 名前 | 説明 |
|-|-|
| [gr_sim_bridge](./node/gr_sim_bridge.md) | サブノードで購読したトピックをUDPパケットに変換してgrSimに送信する |
| [gr_sim_bridge_robot_subscriber](./node/gr_sim_bridge_robot_subscriber.md) (sub node) | ロボットに関するトピックを受信する |
| [gr_sim_bridge_ball_subscriber](./node/gr_sim_bridge_ball_subscriber.md) (sub node) | ボールに関するトピックを受信する |

## class

| 名前 | 説明 |
|-|-|
| sender_node | [gr_sim_bridge](./node/gr_sim_bridge.md)のclass |
| ball_subscriber_node | [gr_sim_bridge_ball_subscriber](./node/gr_sim_bridge_ball_subscriber.md)(sub node)のclass |
| robot_subscriber_node | [gr_sim_bridge_robot_subscriber](./node/gr_sim_bridge_robot_subscriber.md)(sub node)のclass |
| expanded_node | rclcpp::Nodeに機能を追加したNode class |
| expanded_sub_node | rclcpp::Node::SharedPtrに機能を追加したNode class |
| node_expander | rclcpp::Nodeに機能を追加する |
