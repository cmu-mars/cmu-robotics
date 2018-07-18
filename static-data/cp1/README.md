## Static data

* The [map](https://github.com/cmu-mars/cp1_base/blob/master/cp1_base/maps/cp1_map.json), including way point locations and locations of charging
  stations.
* A set of [software components](https://github.com/cmu-mars/model-learner/blob/master/conf/conf.json) (configuration options) that can be used by the
  robot.
* A power model from a set of predefined [power models](https://github.com/cmu-mars/cp1_base/tree/master/cp1_base/power_models) that are inherently
  different from each other and they simulate different power consumptions
  of the robot.

## List of log files

A list of paths to log files that we want to be saved from every test run for postmortem analyses:

* Standard output of `TA`
* Standard error of `TA`
* `/home/mars/cp1/\*`
* `/usr/src/app/access.log`
* `/home/mars/rainbow.log`
* `/home/mars/.ros/logs/latest/\*`
* `/home/mars/logs/\*`
