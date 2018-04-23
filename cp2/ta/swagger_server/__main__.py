#!/usr/bin/env python3

import connexion
import sys
from swagger_server import encoder
from swagger_client import DefaultApi

if __name__ == '__main__':
    # Parameter parsing, to set up TH
    if len(sys.argv) != 2:
      print ("No URI for TH passed in!")
      sys.exit(1)

    th_uri = sys.argv[1]

    # Set up TA server and logging
    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = encoder.JSONEncoder
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
    logger.debug("connecting to the TH")
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
        logger.debug("Failed to connect with th")
        logger.debug(traceback.format_exc())
        raise e

    ## todo: currently we don't even do anything with ready! see
    ## https://github.mit.edu/brass/cmu-robotics/issues/39

    # Start the TA listening
    logger.debug("running the TA")
    app.run(port=5000, host='0.0.0.0')
