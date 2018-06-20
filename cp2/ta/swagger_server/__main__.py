#!/usr/bin/env python3
import time
import connexion
import sys
import logging
import traceback
import threading
import argparse
from swagger_server import encoder
from swagger_client import DefaultApi

from orchestrator import Orchestrator
from orchestrator.exceptions import *

from swagger_server import config
from swagger_server.converters import *
from swagger_client.models.parameters_1 import Parameters1
from swagger_client.models.parameters import Parameters
from swagger_server.models.error_error import ErrorError

logger = logging.getLogger("cp2ta")  # type: logging.Logger
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.NullHandler())


cli = argparse.ArgumentParser('CP2 TA')
cli.add_argument('--th-url',
                 type=str,
                 default='http://cp2_th:5001')
cli.add_argument('--boggart-url',
                 type=str,
                 default='http://boggart:6000')
cli.add_argument('--bugzoo-url',
                 type=str,
                 default='http://bugzoo:6060')
cli.add_argument('--rooibos-url',
                 type=str,
                 default='http://rooibos:888')
cli.add_argument('--threads',
                 type=int,
                 default=8)


def shutdown():
    """
    Responsible for ensuring that all resources are deallocated before the
    TA is closed.
    """
    logger.info("Cleaning up resources")
    try:
        if config.orc:
            config.orc.shutdown()
        else:
            logger.info("No resources left to clean-up: orchestrator was never started")
    except Exception:
        logger.exception("Failed to safely shutdown repair stack.")
    logger.info("Cleaned up resources")


def main():
    args = cli.parse_args()
    th_uri = args.th_url
    boggart_url = args.boggart_url
    bugzoo_url = args.bugzoo_url
    rooibos_url = args.rooibos_url
    threads = args.threads

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
    log_to_stdout.setLevel(logging.INFO)
    log_to_file = logging.FileHandler('/var/log/ta/ta.log', 'w')
    log_to_file.setFormatter(log_formatter)

    logger.setLevel(logging.DEBUG)
    logger.addHandler(log_to_stdout)
    logger.addHandler(log_to_file)

    def setup_logger(name: str,
                     stdout: bool = False,
                     level: int = logging.DEBUG
                     ) -> None:
        lgr = logging.getLogger(name)
        lgr.setLevel(level)
        lgr.addHandler(log_to_file)
        if stdout:
            lgr.addHandler(log_to_stdout)

    # setup_logger('werkzeug')
    setup_logger('orchestrator', stdout=True)
    setup_logger('darjeeling', stdout=True)
    setup_logger('boggart', level=logging.ERROR)
    setup_logger('bugzoo', level=logging.ERROR)
    # setup_logger('rooibos', level=logging.WARNING)

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
        outcome_s = ({
            'NO_REPAIR': 'no-repair',
            'PARTIAL_REPAIR': 'partial-repair',
            'COMPLETE_REPAIR': 'complete-repair'
        })[outcome.name]  # type: str
        response = Parameters1(outcome=outcome_s,
                               running_time=runtime,
                               num_attempts=attempts,
                               pareto_set=[patch2ca(x) for x in pareto],
                               log=[patch2ca(x) for x in log])
        shutdown()
        thApi.done_post(response)

    def error_cb(err_code, msg):
        shutdown()
        thApi.error_post(ErrorError(kind=err_code, message=msg))

    config.orc = Orchestrator(boggart_url,
                              bugzoo_url,
                              rooibos_url,
                              progress_cb,
                              done_cb,
                              error_cb,
                              threads=threads)

    def fail_hard(s):
        shutdown()
        logger.debug(s)
        thApi.error_post(Parameters(s))
        raise Exception(s)

    # start the sequence diagram: post to ready to get configuration data
    def send_ready():
        time.sleep(5)
        try:
            logger.debug("posting to /ready")
            ready_resp = thApi.ready_post()
            logger.debug("received response from /ready:")
            logger.debug(str(ready_resp))
        except Exception as e:
            logger.debug("Failed to connect with th")
            logger.debug(traceback.format_exc())
            raise e

    # FIXME for now, we send /ready after a fixed delay
    t = threading.Thread(target=send_ready)
    t.start()

    # Start the TA listening
    logger.debug("running the TA")
    app.run(port=5000, host='0.0.0.0', threaded=True)


if __name__ == '__main__':
    main()
