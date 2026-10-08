#!/bin/bash

source /home/ctuav/myenv/bin/activate
source /opt/ros/jazzy/setup.bash
source /home/ctuav/ctuav_link_cpp_ws/install/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

python3 4_motor.py  