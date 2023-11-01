# [kiks_gr_sim_bridge_ros](../../../../README.md)/[jp](../../jp.md)/[node](../node.md)/gr_sim_bridge_robot_subscription

ロボットに関するトピックを受信する。

## parametor

| 名前 | 型 | デフォルト | 説明 |
|-|-|-|-|
| timeout_duration | float | 0.1 | cmd_velがtimeoutしたと判断するまでの時間[s]。 |
| timeout_callback_count | int | 30 | cmd_velがtimeoutしてからtimeoutのcallbackを実行する回数(rateが0.0の場合はゼロ司令を送信する回数)。 |
| dyanmic_qos.reliability | string | "best_effort" | 動的なtopicの配信/購読を行う際のtopicの信頼度。"best_effort"か"reliable"以外を指定するとエラーとなる。 |

## topic
| 名前 | 型 | 配信/購読 | 説明 |
|-|-|-|-|
| cmd_vel | geometry_msgs/msg/Twist | 購読 | ロボットの移動速度司令。 |
| cmd_spinner | pendulum_msgs/msg/JointState | 購読 | ロボットのスピナー(ドリブルバー)回転速度司令。 |
| cmd_flat_kick | pendulum_msgs/msg/JointState | 購読 | ロボットのフラットキック速度司令。 |
| cmd_chip_kick | pendulum_msgs/msg/JointState | 購読 | ロボットのチップキック速度司令。 |
| cmd_chip_kick | pendulum_msgs/msg/JointState | 購読 | ロボットのチップキック速度司令。 |
| initialpose | geometry_msgs/msg/PoseWithCovarianceStamped | 購読 | ロボットの再配置。 |