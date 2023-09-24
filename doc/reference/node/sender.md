# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/node/sender

## Description
Subscribe the ball by the node that manages this node.

default name : ssl_bridge_vision

## Parameters

#### udp.port
- Type : int64
- Default : 20011
- Description : topic name and child_frame_id of tf

#### udp.gr_sim_address
- Type : string
- Default : "127.0.0.1"
- Description : topic name and child_frame_id of tf

#### sending_freq
- Type : double
- Default : 60.0

- Description : header frame_id of topic and tf

#### yellow_robots
- Type : string array
- Default : {"yellow00", "yellow01", ... , "yellow14", "yellow15"}
- Description : Yellow_team robots("" will result in an invalid robot)

#### blue_robots
- Type : string array
- Default : {"bulue00", "bulue01", ... , "bulue14", "bulue15"}
- Description : Blue_team robots("" will result in an invalid robot)

#### ball.enable
- Type : bool
- Default : true

- Description : header frame_id of topic and tf

## Related

### class
- [sender_node](../class/sender_node.md)

### node
- [receiver](receiver.md)

### executor
- [ssl_vision_bridge](../executor/ssl_vision_bridge.md)

###### &copy; 2023 KIKS