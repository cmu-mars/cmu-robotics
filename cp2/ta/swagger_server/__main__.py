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

logger = logging.getLogger("cp2ta")  # type: logging.Logger
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.NullHandler())


if __name__ == '__main__':
    # Parameter parsing, to set up TH
    if len(sys.argv) != 4:
      print("expected th_uri boggart_url bugzoo_url")
      sys.exit(1)

    th_uri = sys.argv[1]
    boggart_url = sys.argv[2]
    bugzoo_url = sys.argv[3]

    # Set up TA server
    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = encoder.JSONEncoder
    app.add_api('swagger.yaml', arguments={'title': 'CP2'}, strict_validation=True)

    # Setup logging
    log_formatter = \
        logging.Formatter('%(asctime)s:%(name)s:%(levelname)s: %(message)s',
                          '%Y-%m-%d %H:%M:%S')
    log_to_stdout = logging.StreamHandler(sys.stdout)
    log_to_stdout.setFormatter(log_formatter)
    log_to_file = logging.FileHandler('/var/log/ta/ta.log')
    log_to_file.setFormatter(log_formatter)

    logger.setLevel(logging.DEBUG)
    logger.addHandler(log_to_stdout)
    logger.addHandler(log_to_file)

    def setup_logger(name):
        lgr = logging.getLogger(name)
        lgr.setLevel(logging.DEBUG)
        lgr.addHandler(log_to_stdout)
        lgr.addHandler(log_to_file)

    # setup_logger('werkzeug')
    setup_logger('orchestrator')
    setup_logger('boggart')
    setup_logger('bugzoo')
    setup_logger('darjeeling')

    def log_request_info():
        logger.debug('Headers: %s', connexion.request.headers)
        logger.debug('Body: %s', connexion.request.get_data())

    app.app.before_request(log_request_info)

    # Connect to th
    logger.debug("connecting to the TH: %s", th_uri)
    thApi = DefaultApi()
    thApi.api_client.configuration.host = th_uri

    def progress_cb(candidate, pareto):
        from ..swagger_client.models.parameters import Parameters
        thApi.status_post(Parameters(adaptation=patch2ca(candidate),
                                     pareto_set=[ patch2ca(x) for x in pareto ]))

    def done_cb(log, attempts, outcome, pareto, runtime):
        from ..swagger_client.models.parameters_1 import Parameters1
        thApi.done_post(Parameters1(outcome=outcome.name,
                                    running_time=runtime,
                                    num_attempts=attempts,
                                    pareto_set=[ patch2ca(x) for x in pareto ],
                                    log=[ patch2ca(x) for x in log ]))

    def error_cb(err_code, msg):
        from .models.error_error import ErrorError
        thApi.error_post(ErrorError(kind=err_code,message=msg))

    config.orc = Orchestrator(boggart_url, bugzoo_url, progress_cb, done_cb, error_cb)

    def fail_hard(s):
        from ..swagger_client.models.parameters import Parameters
        logger.debug(s)
        thApi.error_post(Parameters(s))
        raise Exception(s)

    ## start the sequence diagram: post to ready to get configuration data
    try:
        logger.debug("posting to /ready")
        ready_resp = thApi.ready_post()
        logger.debug("received response from /ready:")
        logger.debug(str(ready_resp))
    except Exception as e:
        logger.debug("Failed to connect with th")
        logger.debug(traceback.format_exc())
        raise e

    # Start the TA listening
    logger.debug("running the TA")
    app.run(port=5000, host='0.0.0.0', threaded=True)
