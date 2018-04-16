#!/bin/bash
cd /home/mars/catkin_ws
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "~/catkin_ws/devel/setup.bash"
 
exec "$@"
