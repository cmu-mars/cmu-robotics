from swagger_client.models.errorparams import Errorparams
from swagger_client.models.statusparams import Statusparams

import swagger_server.config as config

def send_status(src, code, x=None, y=None):
    ## todo, this is pretty hacky; really we want to make x, y
    ## optional in the API def and only send them if the robot's
    ## been started
    if x = None:
        x = -1.0
    if y = None:
        y = -1.0

    config.logger.debug("sending status %s from %s" % (code,src))
    response = thApi.status_post(Statusparams(status=code,
                                              x = x, ## todo these could be computed instead of taken as args
                                              y = y,
                                              charge = config.battery, # todo may be none?
                                              sim_time = rospy.Time.now().secs))
    config.logger.debug("repsonse from TH to status: %s" % response)

def send_done()
