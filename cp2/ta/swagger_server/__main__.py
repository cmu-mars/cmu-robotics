#!/usr/bin/env python3
import time
import connexion
import sys
import logging
import traceback
import threading
import argparse
import tarfile
import os
import json
import subprocess
from swagger_server import encoder
from swagger_client import DefaultApi

from orchestrator import Orchestrator
from orchestrator.exceptions import *

from swagger_server import config
from swagger_server.converters import *
from swagger_client.models.parameters_1 import Parameters1
from swagger_client.models.parameters import Parameters
from swagger_server.models.error_error import ErrorError

FN_LOG_TA = '/var/log/ta.log'
logger = logging.getLogger("cp2ta")  # type: logging.Logger
logger.setLevel(logging.DEBUG)
log_formatter = \
    logging.Formatter('%(asctime)s:%(name)s:%(levelname)s: %(message)s',
                      '%Y-%m-%d %H:%M:%S')
log_to_stdout = logging.StreamHandler(sys.stdout)
log_to_stdout.setFormatter(log_formatter)
log_to_stdout.setLevel(logging.INFO)
log_to_file = logging.FileHandler(FN_LOG_TA, 'w')
log_to_file.setFormatter(log_formatter)
logger.addHandler(log_to_stdout)
logger.addHandler(log_to_file)

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
    try:
        write_logs_to_s3()
    except Exception:
        logger.exception("Failed to write logs to S3.")


def write_logs_to_s3():
    """
    Collects the log files for the challenge problem and uploads them to S3.
    """
    logger.info("Attempting to write logs to S3")

    # attempt to fetch the UUID for the ECS task
    uuid_arn = "UNKNOWN-ARN-UUID"
    try:
        try:
            ecs_metadata_fn = os.environ['ECS_CONTAINER_METADATA_FILE']
        except KeyError:
            raise Exception("ECS_CONTAINER_METADATA_FILE not found in environment.")

        try:
            with open(ecs_metadata_fn, 'r') as f:
                jsn = json.load(f)
                uuid_arn = jsn['TaskARN'].split('/')[2]
        except IOError:
            msg = "failed to open metadata file [{}]".format(ecs_metadata_fn)
            raise Exception(msg)
        except KeyError:
            raise Exception("failed to find TaskARN property in metadata file")

    except Exception as e:
        logger.exception("failed to find TaskARN UUID: %s", e)
        logger.warning("using fallback ARN UUID: %s", uuid_arn)

    # ensure that TA logs are flushed to disk
    log_to_file.flush()

    # create a gzipped archive of the logs
    log_archive_fn = '/tmp/{}.tar.gz'.format(uuid_arn)
    # log_archive_fn = '/borko/{}.tar.gz'.format(uuid_arn)
    logger.info("writing log files to compressed archive: %s", log_archive_fn)
    try:
        component_log_files = [
            'ta.log', 'bugzood.log', 'boggartd.log'
        ]
        with tarfile.open(log_archive_fn, 'w:gz') as tf:
            for component_log_fn in component_log_files:
                try:
                    tf.add('/var/log/{}'.format(component_log_fn),
                           component_log_fn)
                except IOError:
                    logger.exception('failed to write %s to archive',
                                     component_log_fn)

    except Exception:
        logger.exception("failed to write log files to compressed archive")
        raise
    logger.info("finished writing log files to compressed archive")

    # upload archive to S3 bucket via the AWS CLI
    url_bucket = "s3://dev-cmur-logs/"
    cmd = "aws s3 cp {} {}".format(log_archive_fn, url_bucket)
    logger.info("Using the following AWS S3 command: %s", cmd)
    try:
        p = subprocess.Popen(cmd,
                             shell=True,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT,
                             universal_newlines=True)
        out, errs = p.communicate()
        if p.returncode != 0:
            raise subprocess.CalledProcessError(p.returncode, cmd, out, errs)

    except subprocess.CalledProcessError as e:
        logger.exception("failed to upload logs to S3: command exited with code [%d]: %s",
                         e.returncode, e.output)
        raise
    except Exception:
        logger.exception("failed to upload logs to S3: unexpected failure.")
        raise

    logger.info("Finished writing logs to S3")


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
    setup_logger('kaskara', level=logging.DEBUG)
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
