#!/usr/bin/env python3

import subprocess

import sys
import connexion
from .encoder import JSONEncoder
import logging
import traceback
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import Float64
from urllib.parse import urlparse

from gazebo_interface import GazeboInterface
from rainbow_interface import RainbowInterface

import threading
import requests
import time
import random
import json
import os
import shutil
import time

from swagger_client import DefaultApi
from swagger_client.models.inline_response_200 import InlineResponse200
from swagger_client.models.parameters import Parameters
from swagger_client.models.parameters_1 import Parameters1
from swagger_client.models.parameters_2 import Parameters2
from swagger_client.models.collision_data import CollisionData

from cp3 import CP3
import swagger_server.config as config
import swagger_server.comms as comms

import swagger_server.resources as resources

if __name__ == '__main__':
    # Command line argument parsing
    if len(sys.argv) != 2:
      print ("No URI for TH passed in!")
      sys.exit(1)

    th_uri = sys.argv[1]

    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = JSONEncoder ## this may be busted; see CP2. depends on codegen version
    app.add_api('swagger.yaml', arguments={'title': 'CP3'}, strict_validation=True)

    ## capture the ambient logger
    logger = logging.getLogger('werkzeug')
    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler(os.path.expanduser('~/logs/TA_access.log'))
    logger.addHandler(handler)
    config.logger = logger

    ## log every HTTP request we see
    def log_request_info():
       logger.debug('Headers: %s', connexion.request.headers)
       logger.debug('Body: %s', connexion.request.get_data())

    app.app.before_request(log_request_info)

    ## build the TH API object from the client stubs
    thApi = DefaultApi()
    thApi.api_client.configuration.host = th_uri
    config.thApi = thApi

    def fail_hard(s):
        logger.debug(s)
        comms.save_ps("error-failhard")

        ## if we at least have the UUID, then try to sequester.
        if config.uuid and config.th_connected:
            comms.sequester()

        if config.th_connected:
            thApi.error_post(Parameters(s))
        raise Exception(s)

    ## record the resources to log
    resources.report_system_resources(logger)
    resources.report_resource_limits(logger)

    ## grab the UUID from the TaskARN, per LL advice
    ## start the sequence diagram: post to ready to get configuration data
    try:
        logger.debug("posting to /ready")
        ready_resp = thApi.ready_post()
        config.th_connected = True
        logger.debug("recieved response from /ready: %s" % ready_resp)
    except Exception as e:
        ## this isn't a call to fail_hard because the TH isn't
        ## responding at all; we have to hope that LL notices the log
        ## output and that this happens only very rarely if at all

        ## note that these logs will not get sequestered
        logger.debug("Failed to connect with th")
        logger.debug(traceback.format_exc())
        config.th_connected = False
        with open(os.path.expanduser(sys.argv[1])) as ready:
            data = json.load(ready)
            ready_resp = InlineResponse200(data["start-loc"], data["target-loc"], data["use-adaptation"], data["start-configuration"], data["utility-function"])
            logger.info("started TA in disconnected mode")

    ## if we get a message from ready, that means we're in the LL
    ## environment and should set up log sequestration
    if config.th_connected:
        ecs_meta = os.environ.get('ECS_CONTAINER_METADATA_FILE')

        if not ecs_meta:
            fail_hard('ECS_CONTAINER_METADATA_FILE not defined; cannot sequester logs')

        ## config.uuid = (subprocess.check_output("cat $ECS_CONTAINER_METADATA_FILE | jq -r '.TaskARN' | cut -d '/' -f2")).strip()
        config.uuid = (subprocess.check_output(os.path.expanduser("~/aws_uuid.sh"))).strip()

        if (not config.uuid) or (len(config.uuid) == 0):
            fail_hard("uuid undefined; cannot sequester logs")


    config.use_adaptation = ready_resp.use_adaptation

    ## build CP object without a gazebo instance, need to set later
    cp = CP3(None, ready_resp.utility_function + '-' + ready_resp.start_configuration)
    config.cp = cp

    ## check dynamic invariants on ready message
    if ready_resp.start_loc == ready_resp.target_loc:
        fail_hard("malformed response from ready: start_loc is target_loc")

    if not (cp.map_server.is_waypoint(ready_resp.target_loc) and
            cp.map_server.is_waypoint(ready_resp.start_loc)):
        fail_hard("response from /ready includes invalid waypoint names")

    ## todo: will this update config as well?
    cp.start = ready_resp.start_loc
    cp.target = ready_resp.target_loc

    ## once the response is checked, write it to ~/ready
    logger.debug("writing checked /ready message to ~/ready")
    fo = open(os.path.expanduser('~/ready'), 'w')
    fo.write('%s' %ready_resp) #todo: this may or may not be JSON; check once we can run it
    fo.close()

    launch_file = ready_resp.start_configuration


    ## todo: this is possibly unnecesscary if we renamed the aruco
    ## file to match the start-configuration string and then just
    ## trust the static checking that this response will be well
    ## formed. the files exist in cp3_base/cp3_base/launch
    if(ready_resp.start_configuration == "aruco-camera"):
        config.nodes = ["aruco-camera"]
        config.sensors = ["camera"]
        launch_file = "aruco-kinect"
    else:
        c = launch_file.split("-")
        config.nodes = [launch_file]
        config.sensors = [c[1]]
    # Check if using marker in world
    if "USE_STATIC_MARKERS" in os.environ.keys():
    	if int(os.environ["USE_STATIC_MARKERS"]) == 1:
            print("Using the world that has the markers statically placed")
            wo_markers=os.path.expanduser("~/catkin_ws/src/cp3_base/worlds/cp3.world")
            w_markers=os.path.expanduser("~/catkin_ws/src/cp3_base/worlds/cp3-markers.world")
            shutil.copy(wo_markers, os.path.expanduser("~/catkin_ws/src/cp3_base/worlds/cp3-orig.world"))
            shutil.copy(w_markers, wo_markers)

    logger.debug("launching cp3-%s.launch" % launch_file)
    rl_child = subprocess.Popen(["roslaunch", "cp3_base", "cp3-" + launch_file + ".launch"],
                                stdin=None,
                                stdout=None,
                                stderr=None) ## todo: unsure about
                                             ## these redirects; see
                                             ## if we like what
                                             ## happens. otherwise can
                                             ## redirect to any file
                                             ## handle instead and
                                             ## capture the logs there
    ## Give ros some time to start
    time.sleep(5)
    ## make this module a ros node so that we can subscribe to topics
    logger.debug("initializing cp3_ta ros node")
    rospy.init_node("cp3_ta")

    ## todo: instead of sleeping, listen to a topic for whatever indicated "odom recieved"
    logger.debug("waiting for move_base (emulates watching for odom_recieved)")
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    ## todo: Ian - should this wait for some period (e.g., timeout=60s), otherwise we will wait forever
    move_base_started = move_base.wait_for_server()
    if not move_base_started:
        fail_hard("fatal error: navigation stack has failed to start")

    print ("Starting up Gazebo interface")
    try:
        gazebo = GazeboInterface(0,0) ## todo: unsure what these args mean but they appear in cli.py
        ## todo: CP3.convert_to_class(cp) ## appears a lot in cli.py but i don't know what it means
        ## todo: this could be totally busted
        cp.gazebo = gazebo
        cp.init()
        start_coords = cp.map_server.waypoint_to_coords(ready_resp.start_loc)
        direction = cp.instruction_server.get_start_heading(ready_resp.start_loc, ready_resp.target_loc, ready_resp.utility_function + '-' + ready_resp.start_configuration)
        gazebo.set_turtlebot_position(start_coords['x'], start_coords['y'], 0)
        # Turn off lights
        for l in config.lights_off:
            gazebo.enable_light(l, False)
    except Exception as e:
        fail_hard("failed to connect to gazebo: %s" % e)

    ## todo: for RR3, do things with  utility function
    if not cp.wait_for_odom(30):
    	fail_hard("failed to set robot position")

    if ready_resp.use_adaptation:
        try:
            rainbow_log = open(os.path.expanduser("~/logs/rainbow.log"),'w')
            rainbow = RainbowInterface()
            ##todo: Ian: be careful copying this for cp1
            rainbow.launchRainbow("cp3", rainbow_log)
            ok = rainbow.startRainbow()
            if not ok:
                fail_hard("did not connect to rainbow in a timely fashion")
        except Exception as e:
            fail_hard("failed to connecto to rainbow: %s " %e)

    ## subscribe to rostopics
    def energy_cb(msg):
        """call back to update the global battery state from the ros topic"""
        ## todo: Ian: This is now float -- check if we need to convert to Int64
        config.battery = int(msg.data)

    sub_voltage = rospy.Subscriber("/energy_monitor/energy_level", Float64, energy_cb)

    ## set the initial plan (in A and B this won't change)
    config.plan = cp.instruction_server.get_path(ready_resp.start_loc,ready_resp.target_loc,ready_resp.utility_function + '-' + ready_resp.start_configuration)
    if (len(config.plan) == 0): # We didn't find a plan for this configuration
        fail_hard('did not find a starting plan for this configuration')

    if not ready_resp.use_adaptation:
        def worker():
            rospy.sleep(5)
            comms.send_status("__main__", "live", "CP3 TA ready to recieve inital perturbs and start in non-adaptive case")

        t = threading.Thread(target=worker)
        t.start()

    def config_updater(sensors, nodes):
        # Note, there is an error here because nodes should just contain
        # amcl or arcuo, but the spec says it is a combo
        #config.logger.debug("%s, %s" %(sensors,nodes))
        old = config.nodes

        config.nodes = []

        if sensors is not None and len(sensors) != 0:
            for i in nodes:
                if 'kinect' in sensors:
                    config.nodes.append(i + "-kinect")
                elif 'lidar' in sensors:
                    config.nodes.append(i + "-lidar")
                elif 'camera' in sensors:
                    config.nodes.append(i + "-camera")
                else:
                    config.node.append(i)
        else:
            # if there is no sensor, that will be indicated in sensors
            for i in nodes:
                config.nodes.append(i + "-kinect") if i != 'aruco' else 'aruco-camera'
        config.sensors = []
        config.sensors.extend(list(sensors))
        if (("amcl-kinect" in config.nodes or "amcl-lidar" in config.nodes) and ("amcl-kinect" not in old and "amcl-lidar" not in old)) or (("mrpt-kinect" in config.nodes or "mrpt-lidar" in config.nodes) and ("mrpt-kinect" not in old and "mrpt-lidar" not in old)):
            rospy.sleep(10)
            logger.debug("Publishing robot pose for map after waiting 10 seconds")
            cp.gazebo.publish_amcl()



    cp.track_config(config_updater)

    logger.debug("starting TA REST interface")

    print("Starting the TA webserver")
    app.run(port=5000, host='0.0.0.0')
