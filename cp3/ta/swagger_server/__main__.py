#!/usr/bin/env python3

import configparser
import sys
import connexion
from .encoder import JSONEncoder
import logging
import traceback
import rospy
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
          if response.start_loc == response.target_loc:
              logger.debug("malformed response from ready: start_loc is target_loc; posting to error")
              thApi.error_post(Parameters("ready error", "start_loc is target_loc"))
        except Exception as e:
          logger.error(traceback.format_exc())
          logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')
        try:
          logger.debug("Sending status")
          response = thApi.status_post(Parameters1("adapted", "Test for status", 55, ["l1", "l2", "l3"], ["MOVEBASE", "AMCL"], ["KINECT_ALL"]))
        except Exception as e:
          logger.error(traceback.format_exc())
          logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')

        wait_time = random.randint(5,60)
        print ('TA sleeping for ' + str(wait_time) + 's before sending done')
        time.sleep(wait_time)

        try:
          logger.debug("Sending done")
          response = thApi.done_post(Parameters2(14.5, 25.9, 72, [72], 2500))
        except Exception as e:
          logger.error(traceback.format_exc())
          logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')

    print ('Starting fake semantics')
    thread = threading.Thread(target=fake_ta)
    thread.start()

if __name__ == '__main__':

    # Command line argument parsing
    if len(sys.argv) != 2:
      print ("No URI for TH passed in!")
      sys.exit(1)

    th_uri = sys.argv[1]

    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = JSONEncoder
    app.add_api('swagger.yaml', arguments={'title': 'CP3'}, strict_validation=True)

    logger = logging.getLogger('werkzeug')
    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler('access.log')
    logger.addHandler(handler)
    def log_request_info():
       logger.debug('Headers: %s', connexion.request.headers)
       logger.debug('Body: %s', connexion.request.get_data())

    app.app.before_request(log_request_info)


    thApi = DefaultApi()
    thApi.api_client.host = th_uri
    try:
      thApi.error_post(Parameters("Test Error", "This is a test error post to th"))
    except Exception as e:
      logger.debug("Failed to connect with th")
      logger.debug(traceback.format_exc())

    rospy.init_node ("cp3_ta")

    print ("Starting up Gazebo interface")
    try:
      gazebo = GazeboInterface()
    except Exception as e:
      logger.error('Fatal: gazebo did not start up: %s' %e)
      thApi.error_post(Parameters("Gazebo Error", "Fatal: failed to connect to gazebo: %s" %e))
      raise
    print ("Started Gazebo Interface")

    fake_semantics(thApi,5000)

    logger.debug("Starting TA")
    app.run(port=5000, host='0.0.0.0')
