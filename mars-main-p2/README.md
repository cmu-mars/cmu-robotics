# mars-main-p15

this repo glues together the repos for each subsystem in the catkin
workspace and holds everything that isn't in one such subsystem:

* https://github.com/cmu-mars/brasscomms
* https://github.com/cmu-mars/ros-topic-probe-p15
* https://github.com/cmu-mars/notifications-p15
* https://github.com/cmu-mars/ig-action-server-p15
* https://github.com/cmu-mars/cp-gazebo-p15

together, this represents "phase 1.5", which is the P1 deliverable running
on kinetic and inside docker rather than indigo and vagrant.

## Installation

To install, simply run:

```
./build.sh
```

This will build individual Docker images for each of the logical components
within the MARS project. Note that there is no need to clone the associated
repositories for each component; the script ensures that the latest version
of each component is built.

## Visualisation

To connect your local machine to the Gazebo server running inside the cluster,
run the following.

```
gzclient
```
