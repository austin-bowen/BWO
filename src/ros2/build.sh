#!/bin/bash

rosdep install -i --from-paths src --rosdistro dashing -y && colcon build --packages-select-regex "bwo.*"
echo Now run: . install/setup.bash
