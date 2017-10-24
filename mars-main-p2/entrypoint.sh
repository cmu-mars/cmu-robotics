#!/bin/bash
cd /home/mars/catkin_ws
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source devel/setup.bash
 
exec "$@"
