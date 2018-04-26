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

import swagger_server.config as config

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
