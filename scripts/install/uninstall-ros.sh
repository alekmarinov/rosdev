#!/bin/sh

sudo apt remove -y ~nros-humble-* && sudo apt autoremove -y
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update -y
sudo apt autoremove -y
sudo apt upgrade -y
sed -i '/ROS_HOME/d' ~/.bashrc
