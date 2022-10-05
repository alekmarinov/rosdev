#!/bin/bash
ROS_DISTRO=humble
ROS_HOME=/opt/ros/$ROS_DISTRO

run() {
    echo "$*"
    sudo $*
    if [[ $? != 0 ]]; then
        echo "Failed: $*" >> /dev/stderr
        exit 1
    fi
}

run apt update -y
run apt install -y software-properties-common
run add-apt-repository -y universe
run apt update -y
run apt install -y curl gnupg lsb-release
run curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
run apt update -y
run apt upgrade -y
run apt install -y ros-$ROS_DISTRO-desktop
source "$ROS_HOME/setup.bash"
run sudo chown -R "$USER:$USER" "$ROS_HOME"
echo "Updating $HOME/.bashrc"
sed -r -i '/ROS_HOME|ROS_DISTRO/d' ~/.bashrc
echo "export ROS_HOME=\"$ROS_HOME\"" >> ~/.bashrc
echo "export ROS_DISTRO=$ROS_DISTRO" >> ~/.bashrc
echo "source \"\$ROS_HOME/setup.bash\"" >> ~/.bashrc
