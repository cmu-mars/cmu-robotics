#!/usr/bin/env python3

import connexion
import sys

import logging
import rospy
import traceback
from urllib.parse import urlparse

import threading
import requests
import time
import random

from .encoder import JSONEncoder
from swagger_client.rest import ApiException
from swagger_client import DefaultApi
from swagger_client.models.parameters import Parameters
from swagger_client.models.parameters_1 import Parameters1
from swagger_client.models.parameters_2 import Parameters2
from swagger_client.models.candidate_adaptation import CandidateAdaptation
from swagger_client.models.test_outcome import TestOutcome
from swagger_client.models.compilation_outcome import CompilationOutcome
from swagger_client.models.degradation import Degradation
from swagger_client.models.test_qo_s import TestQoS

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
        except Exception as e:
          logger.error(traceback.format_exc())
          logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')

        try:      
          logger.debug("Sending status")
          response = thApi.status_post(Parameters1(CandidateAdaptation("diff", CompilationOutcome(20.1,True), (), [TestOutcome("test1", 50.0, False, False, True, qos=TestQoS('','',''))]), [CandidateAdaptation("diff", CompilationOutcome(20.1,True), (), [TestOutcome("test1", 50.0, False, False, True, qos=TestQoS('','',''))])]))
        except Exception as e:
          logger.error(traceback.format_exc())
          logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')
        
        wait_time = random.randint(5,60)
        print ('TA sleeping for ' + str(wait_time) + 's before sending done')
        time.sleep(wait_time)
        
        try:      
          logger.debug("Sending done")
          response = thApi.done_post(
            Parameters2("CompleteRepair",50.0,2, 
              [CandidateAdaptation("diff", 
                CompilationOutcome(20.1,True), 
                (), 
                [TestOutcome("test1", 50.0, False, False, True, TestQoS('','',''))])], 
              [CandidateAdaptation("diff", 
                CompilationOutcome(20.1,True), 
                (), 
                [TestOutcome("test1", 50.0, False, False, True, TestQoS('','',''))])]))
        except Exception as e:
          logger.error(traceback.format_exc())
          logger.error('Fatal: could not connect to TH -- see last logger entry to determine which one')
     
    print ('Starting fake semantics')
    thread = threading.Thread(target=fake_ta)
    thread.start()     

if __name__ == '__main__':

    # Parameter parsing, to set up TH
    if len(sys.argv) != 2:
      print ("No URI for TH passed in!")
      sys.exit(1)
      
    th_uri = sys.argv[1]
   
    # Set up TA server and logging
    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = JSONEncoder
    app.add_api('swagger.yaml', arguments={'title': 'CP2'}, strict_validation=True)

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
      logger.debug("Sending test NeutralPerturbation to th")
      thApi.error_post(Parameters("NeutralPerturbation", "This is a test error post to th"))  
    except Exception as e:
      logger.debug("Failed to connect with th: %s" %e)
      logger.debug(traceback.format_exc())
 
    #Init me as a node
    rospy.init_node("cp2_ta")
 
    fake_semantics(thApi,5000)
    
    # Start the TA listening
    app.run(port=5000, host='0.0.0.0')
