#!/bin/bash
# ensure overlay install is in scope
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash
# source /sim_ws/install/setup.bash


# then build *and* source your dev workspace
#cd ~/grunt_ws
#colcon build --symlink-install
#source install/setup.bash

exec "$@"   # run whatever CMD the user passed