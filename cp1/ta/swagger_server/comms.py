import rospy

from swagger_client.models.doneparams import Doneparams
from swagger_client.models.statusparams import Statusparams
import swagger_server.config as config


def send_status(src, code, sendxy=True):
    ## todo, this is pretty hacky; really we want to make x, y
    ## optional in the API def and only send them if the robot's
    ## been started
    x = -1.0
    y = -1.0
    if sendxy:
        x , y , ig1 , ig2 = config.bot_cont.gazebo.get_bot_state()


    config.logger.debug("sending status %s from %s" % (code,src))
    response = config.thApi.status_post(Statusparams(status=code,
                                              x=x,
                                              y=y,
                                              charge=config.battery, # todo may be none?
                                              sim_time=rospy.Time.now().secs))
    config.logger.debug("repsonse from TH to status: %s" % response)


def send_done(src, msg, outcome):
    x , y , ig1 , ig2 = config.bot_cont.gazebo.get_bot_state()

    config.logger.debug("sending done from %s" % src)
    response = config.thApi.done_post(Doneparams(x=x,
                                                 y=y,
                                                 charge=config.battery,
                                                 sim_time=rospy.Time.now().secs,
                                                 tasks_finished=config.tasks_finished,
                                                 outcome=outcome,
                                                 message=msg))
    config.logger.debug("repsonse from TH to done: %s" % response)
