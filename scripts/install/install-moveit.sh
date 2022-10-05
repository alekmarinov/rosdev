#!/bin/bash

# Check if env var is present and exits if not
# expect_env ENV_VAR_NAME HINT_MESSAGE
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

# Executes command with options and exit on failure
# run COMMAND OPTIONS...
run() {
    echo "$*"
    $*
    if [[ $? != 0 ]]; then
        echo "Failed: $*" >> /dev/stderr
        exit 1
    fi
}

# Executes command with sudo
# sudorun COMMAND OPTIONS...
sudorun() {
    run sudo $*
}

# Required env vars
expect_env ROS_DISTRO "source /opt/ros/humble/setup.bash"
ROS_HOME=/opt/ros/$ROS_DISTRO
expect_env ROS_VERSION "source $ROS_HOME/setup.bash"

# Installs prerequisites
if [ ! -d /etc/ros/rosdep/sources.list.d/ ]; then
    sudorun rosdep init
fi

sudorun rosdep update
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

# install moveit and tutorials
COLCON_WS=~/ws_moveit2/
run mkdir -p "$COLCON_WS/src"
run cd $COLCON_WS/src
run rm -rf moveit2_tutorials
run git clone https://github.com/ros-planning/moveit2_tutorials -b $ROS_DISTRO --depth 1
run vcs import --force < moveit2_tutorials/moveit2_tutorials.repos
sudorun apt update -y 
sudorun rosdep update
sudorun rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd $COLCON_WS
source $ROS_HOME/setup.sh
run colcon build --mixin release --parallel-workers 2

echo "Updating $HOME/.bashrc"
sed -i '/COLCON_WS/d' ~/.bashrc
echo "export COLCON_WS=$COLCON_WS" >> ~/.bashrc
echo "source \$COLCON_WS/install/setup.bash" >> ~/.bashrc
