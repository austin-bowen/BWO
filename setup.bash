# Run: source setup.bash

echo Sourcing ROS2...
source ~/Projects/ros2_foxy/install/setup.bash

echo Sourcing BWO...
source ~/Projects/BWO/src/ros2/install/setup.bash

echo Setting PYTHONPATH...
PYTHONPATH=~/Projects/BWO/src/python:~/Projects/serial-packets/src/python:$PYTHONPATH
