#!/usr/bin/env python3

import configparser
import sys
import connexion
from .encoder import JSONEncoder
import logging

import rospy

import swagger_client
from swagger_client.rest import ApiException
from swagger_client import Configuration
from swagger_client import DefaultApi
from swagger_client.models.parameters import Parameters
from swagger_client.models.parameters_1 import Parameters1
from swagger_client.models.parameters_2 import Parameters2


if __name__ == '__main__':
#    config = configparser.ConfigParser()
#    config.read('network.conf')

#    try:
#        print("brass TH at %s:%s" % (config.get('TH', 'host'), config.getint('TH', 'port')))
#        print("brass TA at %s:%s" % (config.get('TA', 'host'), config.getint('TA', 'port')))
#    except configparser.NoSectionError:
#        print("malformed config file:\n" % str(e))
#        sys.exit(1)
#    except configparser.NoOptionError:
#        print("malformed connfig file:\n" % str(e))
#        sys.exit(1)

    # Parameter parsing, to set up TH
    if len(sys.argv) != 3:
      print ("No URI for TA or TH passed in!")
      sys.exit(1)
      
    th_uri = sys.argv[1]
    ta_uri = sys.argv[2]
    
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
    config = Configuration()
    config.host = th_uri
    
    thApi = DefaultApi(config)   
    
    
    # Hack: Try sending stuff to TH
    try:
      logger.debug("Sending test parsing-error to th")
      thApi.error_post(Parameters("parsing-error", "This is a test error post to th"))  
    except Exception as e:
      logger.debug("Failed to connect with th")
    
 
    
    try:
      logger.debug("Sending ready");
      response = thApi.ready_post()
      logger.debug('Received response from th/ready:')
      logger.debug ('%s' %response)
      
      logger.debug("Sending status")
      response = thApi.status_post(Parameters1("Adapted", 14.5, 30.5, 0.54, 0.35, 42000, 72, 50))
     
      logger.debug("Sending done")
      response = thApi.status_post(Parameters2(14.5, 25.9, 0.54, 0.35, 4000, 72, 72, [45,72], "made it", "test finished successfully"))
    except Exception as e:
      logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')
    
    # Init me as a node
    rospy.init_node("cp1_ta")
    
    # Start the TA listening
    app.run(port=8080)
