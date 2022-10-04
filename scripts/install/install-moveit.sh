#!/bin/bash

expect_env() {
    name_ref=$1
    value=$(eval "echo \"\$$name_ref\"")
    echo "expect_env $name_ref $value"
    if [[ "$value" == "" ]]; then
        echo "Undefined env $1" >> /dev/stderr
        echo "Try $2" >> /dev/stderr
        exit 1
    fi
}

run() {
    echo "$*"
    $*
    if [[ $? != 0 ]]; then
        echo "Failed: $*" >> /dev/stderr
        exit 1
    fi
}

sudorun() {
    run sudo $*
}

expect_env ROS_VERSION "source /opt/ros/humble/setup.bash"
expect_env ROS_DISTRO "export ROS_DISTRO=hubmle"

if [ ! -d /etc/ros/rosdep/sources.list.d/ ]; then
    sudorun rosdep init
fi
run rosdep update
sudorun apt update -y
sudorun apt dist-upgrade -y
sudorun apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget \
  clang-format

run colcon mixin remove default
run colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
run colcon mixin update default

# install some pip packages needed for testing
run python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

export COLCON_WS=~/ws_moveit2/
# install moveit
if [ ! -d $COLCON_WS/src/moveit2 ]; then
    run cd $COLCON_WS/src
    run git clone https://github.com/ros-planning/moveit2.git -b $ROS_DISTRO
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
    run rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    run cd $COLCON_WS
    run colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
fi
source $COLCON_WS/install/setup.bash

# install tutorials
if [ ! -d $COLCON_WS/src/moveit2_tutorials ]; then
    run cd $COLCON_WS/src
    run git clone https://github.com/ros-planning/moveit2_tutorials -b main --depth 1
    run vcs import < moveit2_tutorials/moveit2_tutorials.repos
    sudorun apt update -y && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    cd $COLCON_WS
    run colcon build --mixin release --parallel-workers 1
fi
sed -i '/COLCON_WS/d' ~/.bashrc
echo "export COLCON_WS=$COLCON_WS" >> ~/.bashrc
echo "source \$COLCON_WS/install/setup.bash" >> ~/.bashrc
