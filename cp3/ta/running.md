## Downloading Docker containers

1. Install docker in your machine
2. Make sure that you can do
   `> docker pull cmumars/p2-cp3`
3. Edit `docker-compose-no-th.yml` to look like the following:

```
services:
  cp3_ta:
    image: cmumars/p2-cp3                      # Refer to the dockerhub container 
    container_name: cp3_ta
    hostname: cp3_ta
    environment:
      - "ROS_HOSTNAME=cp3_ta"
      - "ROS_MASTER_URI=http://cp3_ta:11311"
      - "TA_PORT=${TA_PORT}"
      - "RAINBOW_DEBUG_MODE=0"                 # 0 is not debug, 1 is start rainbow in debug
    volumes:
      - start:/start                           # Mount a directory for ready messages for testing
      - roslogs:/home/mars/.ros/latest
      - logs:/home/mars/logs
    ports:
      - ${TA_PORT}:5000
      - 1044:1044                              # The Java remote debugger connections
    expose:
      - 5000
      - 1044
    command: bash -c ". entrypoint.sh && cd /usr/src/app && ./start_exp.sh ${START}"
```
4. Create a directory `start` in the same directory as the compose file. Also create `roslogs` and `logs`.

5. To run the composition do:
   `TA_PORT=8000 START=start/test-ready docker-compose -f docker-compose-no-th.yml up` 
   If all is successful, you should see the webserver start up
   ```
   ```
   To stop, do Ctrl-c, and also do
   `TA_PORT=8000 docker-compose -f docker-compose-no-th.yml down`
   Note that this kills the machine, so any files that you want on the machine will no longer be there

## Running the example
1. From the host, you should be able to:
   a. Start the robot:`curl -X POST http://localhost:8000/start`. This starts the robot with the mission defined in `start/test-ready.json`.
   b. Perturb the robot software: `curl -X POST http://localhost:8000/perturb/nodefail -d '{"id" : "amcl"}' -H "Content-Type:application/json"`. This kills the node `amcl`. Other valid nodes are `aruco, mrpt`, but it only makes sense to do this if the node is running
   c. Perturb teh robot hardware: `curl -X POST http://localhost:8000/perturb/sensor -d '{"id" : "kinect", "state" : "false"}' -H "Content-Type:application/json"`. This kills the `kinect` sensor. Other valid sensors are `camera, lidar`. Like above, this only makes sense if the robot is using these sensors.
   c. Turn on and off lights: `curl -X POST http://localhost:8000/perturb/light -d '{"id" : "light0", "state" : "false"}' -H "Content-Type:application/json"`

2. You should also be able to find out where the robot is, and the current state, by:
   `curl http://localhost:8000/observe`