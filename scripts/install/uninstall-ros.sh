#!/bin/sh

sudo apt remove -y ~nros-humble-* && sudo apt autoremove -y
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo apt update -y
sudo apt autoremove -y
sudo apt upgrade -y
echo "Updating $HOME/.bashrc"
sed -r -i '/ROS_HOME|ROS_DISTRO/d' ~/.bashrc
