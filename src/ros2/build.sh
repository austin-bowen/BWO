#!/bin/bash

rosdep install -i --from-paths src --rosdistro foxy -y && colcon build --packages-select-regex "bwo.*"
echo Now run: . install/setup.bash
