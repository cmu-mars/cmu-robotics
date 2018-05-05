#!/usr/bin/env python3

import sys
import connexion
#sys.path.append('/usr/src/app')

# from .encoder import JSONEncoder
import logging
import traceback
import os
import json
from multiprocessing import Process, Queue
import subprocess


import rospy
from urllib.parse import urlparse

from operator import eq

import threading
import requests
import time
import random
import actionlib

from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction

from swagger_client.rest import ApiException
from swagger_client import DefaultApi
from swagger_client.models.inline_response_200 import InlineResponse200
from swagger_client.models.errorparams import Errorparams
from swagger_client.models.statusparams import Statusparams

import swagger_server.config as config
import swagger_server.comms as comms
from swagger_server.util import *
from swagger_server.encoder import JSONEncoder


import learner
from robotcontrol.bot_controller import BotController
from robotcontrol.rainbow_interface import RainbowInterface
from robotcontrol.launch_utils import *

if __name__ == '__main__':
    # Parameter parsing, to set up TH
    if len(sys.argv) != 2:
        print("No URI TH passed in!")
        sys.exit(1)

    th_uri = sys.argv[1]
    th_connected = False

    # Set up TA server and logging
    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = JSONEncoder
    app.add_api('swagger.yaml', arguments={'title': 'CP1'}, strict_validation=True)

    # capture the logger
    logger = logging.getLogger('werkzeug')
    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler('access.log')
    logger.addHandler(handler)

    # share logger with endpoints
    config.logger = logger

    def log_request_info():
        logger.debug('Headers: %s', connexion.request.headers)
        logger.debug('Body: %s', connexion.request.get_data())

    app.app.before_request(log_request_info)

    # build the TH API object
    thApi = DefaultApi()
    thApi.api_client.configuration.host = th_uri
    config.thApi = thApi

    def fail_hard(s):
        logger.debug(s)
        if th_connected:
            err = Errorparams(error="other-error", message=s)
            result = thApi.error_post(err)
        raise Exception(s)

    # start the sequence diagram: post to ready to get configuration data
    try:
        logger.debug("posting to /ready")
        ready_resp = thApi.ready_post()
        th_connected = True
        logger.debug("received response from /ready: %s" % ready_resp)
    except Exception as e:
        # this isn't a call to fail_hard because the TH isn't
        # responding at all; we have to hope that LL notices the log
        # output and that this happens only very rarely if at all
        logger.debug("failed to connect with th")
        logger.debug(traceback.format_exc())
        th_connected = False
        ready_file_name = sys.argv[1]
        # Adding test ready info
        with open(os.path.expanduser(ready_file_name)) as ready:
            data = json.load(ready)
            ready_resp = InlineResponse200(level=data["level"], start_loc=data["start-loc"], target_locs=data["target-locs"], power_model=data["power-model"], discharge_budget=data["discharge-budget"])
            logger.info("started TA in disconnected mode")
        # raise e

    print(ready_resp.level)

    # dynamic checks on ready response
    if ready_resp.target_locs == []:
        fail_hard("malformed response from ready: target_locs must not be the empty list")

    if ready_resp.start_loc == ready_resp.target_locs[0]:
        fail_hard("malformed response from ready: start-loc must not be the same as the first item of target-locs")

    if not check_adj(ready_resp.target_locs):
        fail_hard("malformed response from ready: target-locs contains adjacent equal elements")

    # once the response is checked, write it to ~/ready
    logger.debug("writing checked /ready message to ~/ready")
    with open(os.path.expanduser('~/ready'), 'w') as ready_file:
        json.dump(ready_resp, ready_file)

    config.level = ready_resp.level

    if ready_resp.level == "c":
        try:
            learner.Learn.get_true_model()
        except Exception as e:
            logger.debug("parsing raised an exception; notifying the TH and then crashing")
            thApi.error_post(Errorparams(error="parsing-error", message="exception raised: %s" % e))
            raise e

        comms.send_status("__main__", "learning-started", False)
        try:
            result = learner.Learn.start_learning()
        except Exception as e:
            logger.debug("learning raised an exception; notifying the TH and then crashing")
            thApi.error_post(Errorparams(error="learning-error", message="exception raised: %s" % e))
            raise e
        comms.send_status("__main__", "learning-done", False)
        learner.Learn.dump_learned_model()

    # ros launch
    # Init me as a node
    logger.debug("initializing cp1_ta ros node")
    # rospy.init_node("cp1_ta")

    q = Queue()
    p = Process(target=launch_cp1_base, args=(q, 'default'))
    p.start()

    # rl_child = subprocess.Popen(["roslaunch", "cp1_base", "cp1-base-test.launch"],
    #                             stdin=None,
    #                             stdout=None,
    #                             stderr=None)
    # launch_cp1_base()
    init("cp1_ta")

    logger.debug("waiting for move_base (emulates watching for odom_recieved)")
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    move_base_started = False
    ind = 0
    while not move_base_started and ind < 12:
        ind += 1
        move_base_started = move_base.wait_for_server(rospy.Duration.from_sec(10))
        rospy.loginfo("waiting for the action server")

    if not move_base_started:
        fail_hard("fatal error: navigation stack has failed to start")

    # build controller object
    bot_cont = BotController()

    # start tracking battery charge
    bot_cont.gazebo.track_battery_charge()
    bot_cont.level = ready_resp.level

    # subscribe to rostopics
    def energy_cb(msg):
        """call back to update the global battery state from the ros topic"""
        # todo: this may be the wrong format (int vs float)
        config.battery = msg.data
        if msg.data <= 0:
            comms.send_done("energy call back", "", "out-of-battery")

    sub_mwh = rospy.Subscriber("/mobile_base/commands/charge_level_mwh", Float64, energy_cb)

    config.bot_cont = bot_cont

    # check that things are actually waypoint names
    if not bot_cont.map_server.is_waypoint(ready_resp.start_loc):
        fail_hard("name of start location is not a waypoint: %s" % ready_resp.start_loc)

    for name in ready_resp.target_locs:
        if not bot_cont.map_server.is_waypoint(name):
            fail_hard("name of target location is not a waypoint: %s" % name)

    # put the robot in the right place
    start_coords = bot_cont.map_server.waypoint_to_coords(ready_resp.start_loc)
    bot_cont.gazebo.set_bot_position(start_coords['x'], start_coords['y'], 0)

    # start up rainbow if we're adapting, otherwise send the live message directly
    if ready_resp.level == "c":
        try:
            rainbow_log = open(os.path.expanduser("~/rainbow.log"), 'w')
            rainbow = RainbowInterface()
            rainbow.launchRainbow("cp1", rainbow_log)
            ok = rainbow.startRainbow()
            if not ok:
                fail_hard("did not connect to rainbow in a timely fashion")
        except Exception as e:
            fail_hard("failed to connection to rainbow: %s " % e)
    elif th_connected:
        comms.send_status("__main__ in level %s" % ready_resp.level, "live")

    logger.debug("Starting TA REST interface")
    print("Starting TA REST interface")
    app.debug = True
    app.run(host='0.0.0.0', port=5000)
