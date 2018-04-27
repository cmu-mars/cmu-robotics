#!/usr/bin/env python3

import configparser
import sys
import connexion
#sys.path.append('/usr/src/app')
#from swagger_server.encoder import JSONEncoder
from .encoder import JSONEncoder
import logging
import traceback
import os

import rospy
from urllib.parse import urlparse

from operator import eq

import threading
import requests
import time
import random

from rainbow_interface import RainbowInterface

from swagger_client.rest import ApiException
from swagger_client import DefaultApi
from swagger_client.models.errorparams import Errorparams
from swagger_client.models.statusparams import Statusparams

import swagger_server.config as config

from learn import Learn ## todo: this may not work
import cli ## todo: this may not work
from bot_controller import BotController

if __name__ == '__main__':
    # Parameter parsing, to set up TH
    if len(sys.argv) != 2:
      print ("No URI TH passed in!")
      sys.exit(1)

    th_uri = sys.argv[1]

    # Set up TA server and logging
    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = JSONEncoder ## possibly busted; unclear
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

    ## todo: utils module some how? copied from CP3
    def fail_hard(s):
        logger.debug(s)
        thApi.error_post(Errorparams(error="other-error",message=s))
        raise Exception(s)

    ## start the sequence diagram: post to ready to get configuration data
    try:
        logger.debug("posting to /ready")
        ready_resp = thApi.ready_post()
        logger.debug("recieved response from /ready: %s" % ready_resp)
    except Exception as e:
        ## this isn't a call to fail_hard because the TH isn't
        ## responding at all; we have to hope that LL notices the log
        ## output and that this happens only very rarely if at all
        logger.debug("failed to connect with th")
        raise e

    ## dynamic checks on ready response
    if ready_resp.target_locs == []:
        fail_hard("malformed response from ready: target_locs must not be the empty list")

    if ready_resp.start_loc == ready_resp.target_locs[0]:
        fail_hard("malformed response from ready: start-loc must not be the same as the first item of target-locs")

    ## todo: probably in a util file
    def check_adj(l):
        for x , y in zip(l, l[1:]):
            if x == y:
                return False
        return True

    if not check_adj(ready_resp.target_locs):
        fail_hard("malformed response from ready: target-locs contains adjacent equal elements")

    ## once the response is checked, write it to ~/ready
    logger.debug("writing checked /ready message to ~/ready")
    fo = open(os.path.expanduser('~/ready'), 'w')
    fo.write('%s' %ready_resp) #todo: this may or may not be JSON; check once we can run it
    fo.close()

    def send_status(src, code, x=None, y=None):
        ## todo, this is pretty hacky; really we want to make x, y
        ## optional in the API def and only send them if the robot's
        ## been started
        if x = None:
            x = -1.0
        if y = None:
            y = -1.0

        config.logger.debug("sending status %s from %s" % (code,src))
        response = thApi.status_post(Statusparams(status=code,
                                                  x = x, ## todo these could be computed instead of taken as args
                                                  y = y,
                                                  charge = config.battery, # todo may be none?
                                                  sim_time = rospy.Time.now().secs))
        config.logger.debug("repsonse from TH to status: %s" % response)

    if ready_resp.level == "c":
        try:
            Learn.get_true_model()
        except Exception as e:
            logger.debug("parsing raised an exception; notifying the TH and then crashing")
            thApi.error_post(Errorparams(error="parsing-error",message="exception raised: %s" % e))
            raise e

        send_status("__main__", "learning-started")
        try:
            result = Learn.start_learning()
        except Exception as e:
            logger.debug("learning raised an exception; notifying the TH and then crashing")
            thApi.error_post(Errorparams(error="learning-error",message="exception raised: %s" % e))
            raise e
        send_status("__main__", "learning-done")
        Learn.dump_learned_model()

    ## ros launch
    # Init me as a node
    logger.debug("initializing cp1_ta ros node")
    rospy.init_node("cp1_ta")

    cli.init("cp1_ta")
    cli.launch_cp1_base()

    logger.debug("waiting for move_base (emulates watching for odom_recieved)")
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    ## todo: Ian - should this wait for some period (e.g., timeout=60s), otherwise we will wait forever
    move_base_started = move_base.wait_for_server()
    if not move_base_started:
        fail_hard("fatal error: navigation stack has failed to start")

    bot_cont = BotController()
    config.bot_cont = bot_cont

    ## subscribe to rostopics
    def energy_cb(msg):
        """call back to update the global battery state from the ros topic"""
        ## todo: this may be the wrong format (int vs float)
        config.battery = msg.data

    sub_mwh = rospy.Subscriber("/energy_monitor/energy_level_mwh", Float64, energy_cb)

    ## start up rainbow if we're adapting, otherwise send the live message directly
    if ready_resp.level == "c":
        try:
            rainbow_log = open(os.path.expanduser("~/rainbow.log"),'w')
            rainbow = RainbowInterface()
            rainbow.launchRainbow("cp1", rainbow_log)
            ok = rainbow.startRainbow()
            if not ok:
                fail_hard("did not connect to rainbow in a timely fashion")
        except Exception as e:
            fail_hard("failed to connecto to rainbow: %s " %e)
    else:
        send_status("__main__ in level %s" % ready_resp.level, "live")

    logger.debug("starting TA REST interface")
    app.run(host='0.0.0.0',port=5000)
