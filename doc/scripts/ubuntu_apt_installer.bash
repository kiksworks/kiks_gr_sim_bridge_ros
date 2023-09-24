# check distribusion
if [ "$(lsb_release -i)" != "Distributor ID:	Ubuntu" ]
then
  echo unsupported distribusion
  exit 255
fi
# check ubuntu rerease
if [ "$(lsb_release -r)" == "Release:	22.04" ]
then
  # setup ros2 foxy
  ls /opt/ros/humble > /dev/null
  if [ "$?" != "0" ]
  then
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install python3-rosdep
    source /opt/ros/dashing/setup.bash
    sudo rosdep init
    cho "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  fi
  sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions libqt6network6 protobuf-compiler
elif [ "$(lsb_release -r)" == "Release:	20.04" ]
then
  # setup ros2 foxy
  ls /opt/ros/foxy > /dev/null
  if [ "$?" != "0" ]
  then
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install python3-rosdep
    source /opt/ros/dashing/setup.bash
    sudo rosdep init
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
  fi
  sudo apt install -y ros-foxy-desktop ros-dev-tools python3-colcon-common-extensions git libqt6network6 protobuf-compiler
elif [ "$(lsb_release -r)" == "Release:	18.04" ]
then
  # setup ros2 dashing
  ls /opt/ros/dashing > /dev/null
  if [ "$?" != "0" ]
  then
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install python3-rosdep
    source /opt/ros/dashing/setup.bash
    sudo rosdep init
    echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
  fi
  sudo apt install -y ros-dashing-desktop python3-colcon-common-extensions git libqt5network5 protobuf-compiler
else
  echo unsupported ubuntu rerease
  exit 255
fi
# rosdep
rosdep install -i --from-paths "$(dirname "$0")"/../../.
  