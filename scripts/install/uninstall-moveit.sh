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

expect_env COLCON_WS

rm -rf $COLCON_WS
echo "Updating $HOME/.bashrc"
sed -i '/COLCON_WS/d' ~/.bashrc
