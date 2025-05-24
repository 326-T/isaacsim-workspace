#!/bin/bash

source /opt/ros/humble/setup.bash

if [ "$1" == "clean" ]; then
    rm -rf build install log
fi

colcon build --packages-select interfaces spacemouse_driver
source install/setup.bash