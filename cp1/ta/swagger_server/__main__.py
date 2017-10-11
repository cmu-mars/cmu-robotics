#!/usr/bin/env python3

import configparser
import sys
import connexion
from .encoder import JSONEncoder
import logging

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
    app.add_api('swagger.yaml', arguments={'title': 'CP1'}, strict_validation=True)

    logger = logging.getLogger('werkzeug')
    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler('access.log')
    logger.addHandler(handler)

    def log_request_info():
        logger.debug('Headers: %s', connexion.request.headers)
        logger.debug('Body: %s', connexion.request.get_data())

    app.app.before_request(log_request_info)

    app.run(port=8080)
