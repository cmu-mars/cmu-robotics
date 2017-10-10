#!/usr/bin/env python3

import sys
import connexion
from .encoder import JSONEncoder
import logging

if __name__ == '__main__':
    try:
        print("working with brass-th: %s" % sys.argv[1])
    except IndexError:
        print("please provide exactly one argument, an address to the TH")
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
