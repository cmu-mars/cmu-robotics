#!/usr/bin/env python3

import configparser
import sys
import connexion
sys.path.append('/usr/src/app')
from swagger_server.encoder import JSONEncoder
import logging
import traceback
import rospy
import actionlib
#from move_base_msgs.msg import MoveBaseAction

from gazebo_interface import GazeboInterface

from swagger_client import DefaultApi
from swagger_client.models.inline_response_200 import InlineResponse200
from swagger_client.models.parameters import Parameters
from swagger_client.models.parameters_1 import Parameters1
from swagger_client.models.parameters_2 import Parameters2

if __name__ == '__main__':
#    config = configparser.ConfigParser()
#    config.read('network.conf')
    print ("The TA and TH should come now")
    print(sys.argv[1:])
#    try:
#        print("brass TH at %s:%s" % (config.get('TH', 'host'), config.getint('TH', 'port')))
#        print("brass TA at %s:%s" % (config.get('TA', 'host'), config.getint('TA', 'port')))
#    except configparser.NoSectionError:
#        print("malformed config file:\n" % str(e))
#        sys.exit(1)
#    except configparser.NoOptionError:
#        print("malformed connfig file:\n" % str(e))
#        sys.exit(1)

    if len(sys.argv) != 3:
      print ("No URI for TA or TH passed in!")
      sys.exit(1)
      
    th_uri = sys.argv[1]
    ta_uri = sys.argv[2]

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
    

    thApi = DefaultApi();
    thApi.api_client.host = th_uri
    try:
      thApi.error_post(Parameters("Test Error", "This is a test error post to th"))  
    except Exception as e:
      logger.debug("Failed to connect with th")
      logger.debug(traceback.format_exc())

   

    rospy.init_node ("cp3_ta")
#    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
#    move_base_started = move_base.wait_for_server(rospy.Duration(30.0))
#    if not move_base_started:
#      logger.error ('Fatal: move_base did not start?')
#      thApi.error_post(Parameters("MoveBase Error", "Fatal: failed to wait for move_base"))
    rospy.sleep(30.0)
    print ("Starting up Gazebo interface")
    try:
      gazebo = GazeboInterface()
      gazebo.set_turtlebot_position(19.8, 58.8, 0);
    except Exception as e:
      logger.error('Fatal: gazebo did not start up: %s' %e)
      thApi.error_post(Parameters("Gazebo Error", "Fatal: failed to connect to gazebo: %s" %e))
      raise
    print ("Started Gazebo Interface")
    
   
    try:
      logger.debug("Sending ready");
      response = thApi.ready_post()
      logger.debug('Received response from th/ready:')
      logger.debug ('%s' %response)
    except Exception as e:
      logger.error(traceback.format_exc())
      logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')
    try:      
      logger.debug("Sending status")
      response = thApi.status_post(Parameters1("adapted", "Test for status", 55, ["l1", "l2", "l3"], ["MOVEBASE", "AMCL"], ["KINECT_ALL"]))
    except Exception as e:
      logger.error(traceback.format_exc())
      logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')
    try: 
      logger.debug("Sending done")
      response = thApi.done_post(Parameters2(14.5, 25.9, 72, [72], 2500))
    except Exception as e:
      logger.error(traceback.format_exc())
      logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')
    
    logger.debug("Starting TA")
    print("Starting TA to listen on 8080")
    app.run(port=8080, host='0.0.0.0')
