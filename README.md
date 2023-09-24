# kiks_gr_sim_bridge_ros

## Description
Bridge ROS2 topic to UDP packets from grSim

## Supported environment
- ubuntu 22.04

note : This package may work on ubuntu20.04 or 18.04, but they are not supported.

## Required packages
- Git ([view installation](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git))
- ROS2 humble ([view installation](https://docs.ros.org/en/humble/Installation.html))
  - ROS2 rclcpp
  - ROS2 geometry_msgs
  - ROS2 pendulum_msgs
  - ROS2 tf2_ros
- colcon ([view installation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html))
- Qt6 or Qt5 (Network)
- Protocol Buffers ([view installation](https://github.com/protocolbuffers/protobuf#protobuf-compiler-installation))

## Installation
1. create work space and source directory
```
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```
note : ros2_ws can be replaced with any name you like.

note : If you have an existing workspace you can use it.

2. clone this repository
```
git clone --recursive https://github.com/kiksworks/kiks_gr_sim_bridge_ros.git kiks_gr_sim_bridge
```
note : If you forget --recursive when cloning, please run "git submodule update --init --recursive".

3. install required packages
```
bash doc/scripts/ubuntu_apt_installer.bash
```
```
source ~/.bashrc
```

4. move to work space directory
```
cd ..
```
5. build
```
colcon build
```

## Run
1. source setup
```
source ~/ros2_ws/install/setup.bash
```
2. run
```
ros2 run kiks_gr_sim_bridge gr_sim_bridge
```

## Document
- [reference](doc/reference/index.md)
<!-- - [guide](doc/guide/index.md) -->

## To contribute
read [CONTRIBUTING.md](CONTRIBUTING.md)

## Contact
- email : kiks.nittc@gmail.com

## License
- GPLv3([view license file](LICENSE))

###### &copy; 2023 KIKS