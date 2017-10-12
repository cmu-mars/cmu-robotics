#!/usr/bin/env python3

import configparser
import sys
import connexion
from .encoder import JSONEncoder
import logging
import requests

import rospy
from gazebo_interface import GazeboInterface
import swagger_client
from swagger_client.rest import ApiException


if __name__ == '__main__':
    config = configparser.ConfigParser()
    config.read('network.conf')

    try:
        print("brass TH at %s:%s" % (config.get('TH', 'host'), config.getint('TH', 'port')))
        print("brass TA at %s:%s" % (config.get('TA', 'host'), config.getint('TA', 'port')))
    except configparser.NoSectionError:
        print("malformed config file:\n" % str(e))
        sys.exit(1)
    except configparser.NoOptionError:
        print("malformed connfig file:\n" % str(e))
        sys.exit(1)

    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = JSONEncoder
    app.add_api('swagger.yaml', arguments={'title': 'CP3'}, strict_validation=True)

    logger = logging.getLogger('werkzeug')
    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler('access.log')
    logger.addHandler(handler)
    thApi = DefaultApi()
    thApi.api_client.host = 'http://' + config.get('TH', 'host') + ":" + config.get('TH', 'port')
    
    thApi.error_post(Parameters("Gazebo Error", "Fatal: failed to connect to gazebo: %s" %e);  

    def log_request_info():
        logger.debug('Headers: %s', connexion.request.headers)
        logger.debug('Body: %s', connexion.request.get_data())

    rospy.init_node ("cp3_ta")
    
    print ("Starting up Gazebo interface")
    try:
      gazebo = GazeboInterface()
    except Exception as e:
      logger.error('Fatal: gazebo did not start up: %s' %e)
      thApi.error_post(Parameters("Gazebo Error", "Fatal: failed to connect to gazebo: %s" %e);  
      raise
    print ("Started Gazebo Interface")
    
    app.app.before_request(log_request_info)
   
    try:
      logger.debug("Sending ready");
      response = thApi.ready_post()
      logger.debug('Received response from th/ready:')
      logger.debug ('%s' %response)
      
      logger.debug("Sending status")
      response = thApi.status_post(Parameters1("Adapted", "Test for status", 55, ["l1", "l2", "l3"], ["MOVEBASE", "AMCL"], ["KINECT_ALL"]))
     
      logger.debug("Sending done")
      response = thApi.status_post(Parameters2(14.5, 25.9, 72, [72], 2500))
    except Exception as e:
      logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')
    
    app.run(port=8080)
