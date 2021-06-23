#!/usr/bin/env bash

echo Starting BWO!

cd "$(dirname "$0")"

# Source ROS stuff
source ../ros2_foxy/install/setup.bash
source ./src/ros2/install/setup.bash

# Setup PYTHONPATH
SERIAL_PACKETS_LIB=../serial-packets/src/python
export PYTHONPATH=./src/python:$SERIAL_PACKETS_LIB:$PYTHONPATH

#exec python -u main.py
ros2 run bwo drive_motors
