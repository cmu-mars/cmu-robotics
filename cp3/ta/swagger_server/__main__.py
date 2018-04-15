#!/usr/bin/env python3

import subprocess

import configparser
import sys
import connexion
from .encoder import JSONEncoder
import logging
import traceback
import rospy
import actionlib
#from move_base_msgs.msg import MoveBaseAction
from urllib.parse import urlparse

from gazebo_interface import GazeboInterface

import threading
import requests
import time
import random

from swagger_client import DefaultApi
from swagger_client.models.inline_response_200 import InlineResponse200
from swagger_client.models.parameters import Parameters
from swagger_client.models.parameters_1 import Parameters1
from swagger_client.models.parameters_2 import Parameters2

from cp3 import CP3
from flask.ext.script import Manager

if __name__ == '__main__':
    # Command line argument parsing
    if len(sys.argv) != 2:
      print ("No URI for TH passed in!")
      sys.exit(1)

    th_uri = sys.argv[1]

    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = JSONEncoder
    app.add_api('swagger.yaml', arguments={'title': 'CP3'}, strict_validation=True)

    manager = Manager(app)

    ## capture the ambient logger
    logger = logging.getLogger('werkzeug')
    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler('access.log')
    logger.addHandler(handler)

    ## log every HTTP request we see
    def log_request_info():
       logger.debug('Headers: %s', connexion.request.headers)
       logger.debug('Body: %s', connexion.request.get_data())

    app.app.before_request(log_request_info)

    ## build the TH API object from the client stubs
    thApi = DefaultApi()
    thApi.api_client.host = th_uri

    def fail_hard(s):
        logger.debug(s)
        thApi.error_post(Parameters(s))
        raise Exception(s)

    ## start the sequence diagram: post to ready to get configuration data
    try:
        logger.debug("posting to /ready")
        ready_resp = thApi.ready_post()
        logger.debug("recieved response from /ready:")
        logger.debug("%s" % resp)
    except Exception as e:
        ## this isn't a call to fail_hard because the TH isn't
        ## responding at all; we have to hope that LL notices the log
        ## output and that this happens only very rarely if at all
        logger.debug("Failed to connect with th")
        logger.debug(traceback.format_exc())
        raise e

    ## check dynamic invariants on ready message
    if ready_resp.start_loc() == ready_resp.target_loc():
        fail_hard("malformed response from ready: start_loc is target_loc")

    ## todo: add checking here to make sure that start_loc and
    ## target_loc are indeed waypoint names once i know what the
    ## waypoint names are. can't (don't want to) use
    ## waypoint_to_coords from CP3 object beloq because i don't want
    ## to build gazebo interface just to check this. could just read
    ## the map json.

    ## once the response is checked, write it to ~/ready
    logger.debug("writing checked /ready message to ~/ready")
    fo = open('~/ready', 'w')
    fo.write('%s', ready_resp) #todo: this may or may not be JSON;
                               #check once we can run it
    fo.close()

    ## todo: this is possibly unnecesscary if we renamed the aruco
    ## file to match the start-configuration string and then just
    ## trust the static checking that this response will be well
    ## formed. the files exist in cp3_base/cp3_base/launch
    launch_file = ""
    if(ready_resp.start_configuration() == "amcl-kinect"):
        launch_file = "amcl-kinect"
    elif(ready_resp.start_configuration() == "amcl-lidar"):
        launch_file = "amcl-lidar"
    elif(ready_resp.start_configuration() == "mprt-kinect"):
        launch_file = "mprt-kinect"
    elif(ready_resp.start_configuration() == "mprt-lidar"):
        launch_file = "mprt-lidar"
    elif(ready_resp.start_configuration() == "aruco-camera"):
        launch_file = "aruco-front"
    else:
        ## todo: this should never happen, given that the static
        ## checks from swagger work as intended. might be able to get
        ## away without it entirely if it's actually dead code.
        fail_hard("ready error: /ready response contained an invalid start_configuration: %s" % ready_resp.start_configuration())

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

    ## make this module a ros node so that we can subscribe to topics
    logger.debug("initializing cp3_ta ros node")
    rospy.init_node("cp3_ta")

    ## todo: instead of sleeping, listen to a topic for whatever indicated "odom recieved"
    logger.debug("waiting for move_base (emulates watching for odom_recieved)")
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base_started = move_base.wait_for_server()
    if not move_base_started:
        fail_hard("fatal error: navigation stack has failed to start")

    print ("Starting up Gazebo interface")
    try:
      gazebo = GazeboInterface(0,0) ## TODO: unsure what these args mean but they appear in cli.py
      cp = CP3(gazebo)
      ## todo: CP3.convert_to_class(cp) ## appears a lot in cli.py but i don't know what it means
      start_coords = cp.map_server.waypoint_to_coords(ready_resp.start_loc())
      gazebo.set_turtlebot_position(start_coords['x'], start_coords['y'], 0)
    except Exception as e:
        fail_hard("failed to connect to gazebo: %s" % e)

    ## todo: for RR2, need to also process use_adaptation and the utility function

    def send_live():
        ## send status live after gazebo interface comes up
        logger.debug("sending live status message")
        ## todo: sending simtime 0 here may be wrong; technically there is
        ## no simtime yet. but it's required in the spec. maybe change the
        ## api so it isn't required?
        live_resp = thApi.status_post(Parameters1("live","CP3 TA ready to recieve inital perturbs and start",0,None,None,None))

    @manager.command
    def runserver():
        app.run(port=5000, host='0.0.0.0')
        send_live()

    logger.debug("starting TA REST interface")
    manager.run()
