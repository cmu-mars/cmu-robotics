from swagger_client.models.doneparams import Doneparams
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
    response = config.thApi.status_post(Statusparams(status=code,
                                              x = x, ## todo these could be computed instead of taken as args
                                              y = y,
                                              charge = config.battery, # todo may be none?
                                              sim_time = rospy.Time.now().secs))
    config.logger.debug("repsonse from TH to status: %s" % response)

def send_done(src, msg, outcome,):
    x , y , ig1 , ig2 = config.bot_cont.gazebo.get_bot_state()

    config.logger.debug("sending done from %s" % src)
    response = config.thApi.done_post(Doneparams(x = x,
                                                 y = y,
                                                 charge=config.battery,
                                                 sim_time=rospy.Time.now().secs,
                                                 tasks_finished=config.tasks_finished,
                                                 outcome=outcome,
                                                 message=msg))
    config.logger.debug("repsonse from TH to done: %s" % response)
