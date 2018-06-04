# Static data for CP3

We have been testing this on a `m5.xlarge` EC2 instance.

The static data for CP3 are the map, which can be found in: https://github.com/cmu-mars/cp3_base/blob/master/cp3_base/maps/cp3.json. This contains
1.  The list of waypoints, their location, and their connectivity
2. The list of lights that are installed, and an indication of those that start in the `off` state.

# Log file locations
In addition to the console trace from the ta, we need the following logs collected:

1. /usr/src/app/access.log
2. /home/mars/rainbow.log
3. /home/mars/.ros/logs/latest/\*
4. /home/mars/das/rainbow-brass/\*.log
5. ~/ps_*.log
