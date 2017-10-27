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

from swagger_client.rest import ApiException
from swagger_client import DefaultApi
from swagger_client.models.parameters import Parameters
from swagger_client.models.parameters_1 import Parameters1
from swagger_client.models.parameters_2 import Parameters2

# Function to mimic wait for ta to be up, send ready, then status, then error, then wait
# and send done
def fake_semantics(thApi, port):
    def fake_ta():
        not_started = True
        while not_started:
            print('Checking to see if TA is up on port ' + str(port))
            try:
                r = requests.get('http://0.0.0.0:' + str(port) + '/')
                print(r.status_code)
                if r.status_code == 200 or r.status_code == 404:
                    print('Server started; Starting to push th')
                    not_started=False
            except:
                print('server not yet started')
            time.sleep(2)

        try:
            logger.debug("Sending ready");
            response = thApi.ready_post()
            logger.debug('Received response from th/ready:')
            logger.debug ('%s' %response)

            # check to make sure that all adjacent waypoints are disequal
            waypoints = response.target_locs

            if not waypoints:
              logger.debug("malformed response from ready: target_locs must not be empty; posting to error")
              thApi.error_post(parameters=Parameters("ready error", "target_locs must not be empty"))

            waypoints.insert(0,response.start_loc)
            if any(map(eq, waypoints, waypoints[1:])):
              logger.debug("malformed response from ready: start_loc @ target_locs has adjacent equal elements; posting to error")
              thApi.error_post(parameters=Parameters("ready error", "start_loc @ target_locs has adjacent equal elements"))
        except Exception as e:
            logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')
            logger.debug(traceback.format_exc())


        try:
            logger.debug("Sending status")
            response = thApi.status_post(parameters=Parameters1("learning-started", 14.5, 30.5, 0.54, 0.35, 42000, 72, 50))
        except Exception as e:
            logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')
            logger.debug(traceback.format_exc())

        wait_time = random.randint(5,60)
        print ('TA sleeping for ' + str(wait_time) + 's before sending done')
        time.sleep(wait_time)

        try:
            logger.debug("Sending done")
            response = thApi.done_post(parameters=Parameters2(14.5, 25.9, 0.54, 0.35, 4000, 72, 72, [45,72], "at-goal", "test finished successfully"))
        except Exception as e:
            logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')
        logger.debug(traceback.format_exc())
    print ('Starting fake semantics')
    thread = threading.Thread(target=fake_ta)
    thread.start()

if __name__ == '__main__':

    # Parameter parsing, to set up TH
    if len(sys.argv) != 2:
      print ("No URI TH passed in!")
      sys.exit(1)

    th_uri = sys.argv[1]

    # Set up TA server and logging
    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = JSONEncoder
    app.add_api('swagger.yaml', arguments={'title': 'CP1'}, strict_validation=True)

    logger = logging.getLogger('werkzeug')
    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler('access.log')
    logger.addHandler(handler)

    def log_request_info():
        logger.debug('Headers: %s', connexion.request.headers)
        logger.debug('Body: %s', connexion.request.get_data())

    app.app.before_request(log_request_info)

    # Connect to th
    thApi = DefaultApi()
    thApi.api_client.host = th_uri

    # Hack: Try sending stuff to TH
    try:
      logger.debug("Sending test parsing-error to th")
      thApi.error_post(parameters=Parameters("parsing-error", "This is a test error post to th"))
    except Exception as e:
      logger.debug("Failed to connect with th")
      logger.debug(traceback.format_exc())



    # Init me as a node
    rospy.init_node("cp1_ta")

    fake_semantics(thApi,5000)

    # Start the TA listening
    app.run(host='0.0.0.0',port=5000)
