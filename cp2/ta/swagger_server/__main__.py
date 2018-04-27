#!/usr/bin/env python3
import connexion
import sys
import logging
import traceback
from swagger_server import encoder
from swagger_client import DefaultApi

from orchestrator import Orchestrator
from orchestrator.exceptions import *

from swagger_server import config
from swagger_server.converters import *

if __name__ == '__main__':
    # Parameter parsing, to set up TH
    if len(sys.argv) != 4:
      print ("expected th_uri hulk_url bugzoo_url")
      sys.exit(1)

    th_uri = sys.argv[1]
    hulk_url = sys.argv[2]
    bugzoo_url = sys.argv[3]

    # Set up TA server and logging
    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = encoder.JSONEncoder
    app.add_api('swagger.yaml', arguments={'title': 'CP2'}, strict_validation=True)

    # FIXME why are we hijacking the werkzeug logger?
    handler_stream = logging.StreamHandler(sys.stdout)
    logger = logging.getLogger('werkzeug')
    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler('access.log')
    logger.addHandler(handler)
    logger.addHandler(logging.StreamHandler())

    logger_orc = logging.getLogger('orchestrator')
    logger_orc.setLevel(logging.DEBUG)
    logger_orc.addHandler(handler_stream)

    def log_request_info():
        logger.debug('Headers: %s', connexion.request.headers)
        logger.debug('Body: %s', connexion.request.get_data())

    app.app.before_request(log_request_info)

    # Connect to th
    logger.debug("connecting to the TH: %s", th_uri)
    thApi = DefaultApi()
    thApi.api_client.configuration.host = th_uri

    def progress_cb(candidate, pareto):
        thApi.status_post(Parameters(adaptation=patch2ca(candidate),
                                     pareto_set=[ patch2ca(x) for x in pareto ]))

    def done_cb(log, attempts, outcome, pareto, runtime):
        thApi.done_post(Parameters1(outcome=outcome.name,
                                    running_time=runtime,
                                    num_attempts=attempts,
                                    pareto_set=[ patch2ca(x) for x in pareto ],
                                    log=[ patch2ca(x) for x in log ]))

    def error_cb(err_code, msg):
        thApi.error_post(ErrorError(kind=err_code,message=msg))

    config.orc = Orchestrator(hulk_url, bugzoo_url, progress_cb, done_cb, error_cb)

    def fail_hard(s):
        logger.debug(s)
        thApi.error_post(Parameters(s))
        raise Exception(s)

    ## start the sequence diagram: post to ready to get configuration data
    try:
        logger.debug("posting to /ready")
        ready_resp = thApi.ready_post()
        logger.debug("recieved response from /ready:")
        logger.debug(str(ready_resp))
    except Exception as e:
        logger.debug("Failed to connect with th")
        logger.debug(traceback.format_exc())
        raise e

    ## todo: currently we don't even do anything with ready! see
    ## https://github.mit.edu/brass/cmu-robotics/issues/39

    # Start the TA listening
    logger.debug("running the TA")
    app.run(port=5000, host='0.0.0.0')
