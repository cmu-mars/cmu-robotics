#! /bin/bash
# Fix up potential executable permissions on python files
source /opt/ros/indigo/setup.bash
cd catkin_ws/src
find . -name "*.py" -exec chmod +x {} \;
catkin_init_workspace
cd ..
catkin_make
