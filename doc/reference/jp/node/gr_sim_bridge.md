# [kiks_gr_sim_bridge_ros](../../../../README.md)/[jp](../../jp.md)/[node](../node.md)/gr_sim_bridge

サブノードで購読したトピックをUDPパケットに変換してgrSimに送信する。

## parametor

| 名前 | 型 | デフォルト | 説明 |
|------------------------------------|---------------------|--------------------------------------|--------------------------------------------|
| udp.port | int | 20011 | grSimとのUDP通信に使用するポート |
| udp.gr_sim_address | string | "127.0.0.1" | grSimがUDPパケットを受け取るアドレス |
| rate | double | 0.0 | grSimにUDPパケットを送信する頻度[hz]。0.0の場合トピックを受信したら非同期でUDPパケットを送信する。 |
| yellow_robots | string array | ["yellow00", "yellow01", ... , "yellow14", "yellow15"] | 黄色チームのロボットの名前空間($ROBOT_NAME)。順番とrobot_idが対応している。""を指定するとそのrobot_idのロボットのブリッジは行わない。"/"を指定するとサブ名前空間なしでブリッジを行う。|
| blue_robots | string array | ["blue00", "blue01", ... , "blue14", "blue15"] | 青色チームのロボットの名前空間($ROBOT_NAME)。順番とrobot_idが対応している。""を指定するとそのrobot_idのロボットのブリッジは行わない。"/"を指定するとサブ名前空間なしでブリッジを行う。|
| dyanmic_qos.reliability | string | "best_effort" | 動的なtopicの配信/購読を行う際のtopicの信頼度。"best_effort"か"reliable"以外を指定するとエラーとなる。 |
