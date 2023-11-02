# [kiks_gr_sim_bridge_ros](../../../../README.md)/[jp](../../jp.md)/[executor](../executor.md)/gr_sim_bridge

ROS2トピックをUDPパケットにブリッジします。

## parametor

| 名前 | 型 | デフォルト | 説明 |
|------------------------------------|---------------------|--------------------------------------|--------------------------------------------|
| udp.port | int | 20011 | grSimとのUDP通信に使用するポート |
| udp.gr_sim_address | string | "127.0.0.1" | grSimがUDPパケットを受け取るアドレス |
| rate | double | 0.0 | grSimにUDPパケットを送信する頻度[hz]。0.0の場合トピックを受信したら非同期でUDPパケットを送信する。 |
| yellow_robots | string array | ["yellow00", "yellow01", ... , "yellow14", "yellow15"] | 黄色チームのロボットの名前空間($ROBOT_NAME)。順番とrobot_idが対応している。""を指定するとそのrobot_idのロボットのブリッジは行わない。"/"を指定するとサブ名前空間なしでブリッジを行う。|
| blue_robots | string array | ["blue00", "blue01", ... , "blue14", "blue15"] | 青色チームのロボットの名前空間($ROBOT_NAME)。順番とrobot_idが対応している。""を指定するとそのrobot_idのロボットのブリッジは行わない。"/"を指定するとサブ名前空間なしでブリッジを行う。|
| ${ROBOT_NAME}.timeout_duration | float | 0.1 | cmd_velがtimeoutしたと判断するまでの時間[s]。 |
| ${ROBOT_NAME}.timeout_callback_count | int | 30 | cmd_velがtimeoutしてからtimeoutのcallbackを実行する回数(rateが0.0の場合はゼロ司令を送信する回数)。 |
| dyanmic_qos.reliability | string | "best_effort" | 動的なtopicの配信/購読を行う際のtopicの信頼度。"best_effort"か"reliable"以外を指定するとエラーとなる。 |

## topic
| 名前 | 型 | 配信/購読 | 説明 |
|-|-|-|-|
| ${ROBOT_NAME}.cmd_vel | geometry_msgs/msg/Twist | 購読 | ロボットの移動速度司令。 |
| ${ROBOT_NAME}.cmd_spinner | pendulum_msgs/msg/JointState | 購読 | ロボットのスピナー(ドリブルバー)回転速度司令。 |
| ${ROBOT_NAME}.cmd_flat_kick | pendulum_msgs/msg/JointState | 購読 | ロボットのフラットキック速度司令。 |
| ${ROBOT_NAME}.cmd_chip_kick | pendulum_msgs/msg/JointState | 購読 | ロボットのチップキック速度司令。 |
| ${ROBOT_NAME}.cmd_chip_kick | pendulum_msgs/msg/JointState | 購読 | ロボットのチップキック速度司令。 |
| ${ROBOT_NAME}.initialpose | geometry_msgs/msg/PoseWithCovarianceStamped | 購読 | ロボットの再配置。 |
| ball.initialpose | geometry_msgs/msg/PoseWithCovarianceStamped | 購読 | ボールの再配置。 |


## node

| 名前 | 説明 |
|-|-|
| gr_sim_bridge | サブノードで購読したトピックをUDPパケットに変換してgrSimに送信する |
| gr_sim_bridge/${ROBOT_NAME} (sub node) | ロボットに関するトピックを受信する |
| gr_sim_bridge/ball (sub node) | ボールに関するトピックを受信する |