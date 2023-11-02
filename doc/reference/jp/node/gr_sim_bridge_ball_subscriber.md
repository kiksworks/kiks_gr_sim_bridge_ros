# [kiks_gr_sim_bridge_ros](../../../../README.md)/[jp](../../jp.md)/[node](../node.md)/gr_sim_bridge_ball_subscription

ロボットに関するトピックを受信する。

## parametor

| 名前 | 型 | デフォルト | 説明 |
|-|-|-|-|
| dyanmic_qos.reliability | string | "best_effort" | 動的なtopicの配信/購読を行う際のtopicの信頼度。"best_effort"か"reliable"以外を指定するとエラーとなる。 |

## topic
| 名前 | 型 | 配信/購読 | 説明 |
|-|-|-|-|
| initialpose | geometry_msgs/msg/PoseWithCovarianceStamped | 購読 | ボールの再配置。 |