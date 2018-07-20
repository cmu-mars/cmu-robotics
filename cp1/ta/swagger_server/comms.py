import rospy
import subprocess
import os
import datetime

from swagger_client.models.errorparams import Errorparams
from swagger_client.models.doneparams import Doneparams
from swagger_client.models.statusparams import Statusparams
import swagger_server.config as config


def sequester():
    if config.th_connected and config.uuid is not None:
        logdirs = ["/home/mars/cp1/",
                   "/usr/src/app/access.log",
                   "/home/mars/rainbow.log",
                   "/home/mars/.ros/logs/latest/",
                   "/home/mars/logs/"
                   ]

        err = False
        for ld in logdirs:
            config.logger.debug("Copying %s with the uuid '%s'" %(ld,config.uuid))
            command = "%s %s %s" %(os.path.expanduser("~/aws_copy.sh"), ld, config.uuid)
            print("Calling command %s" %command)
            res = subprocess.call(command, shell=True)
            if not res == 0:
                err = True

        # if any of the directories can't be copied, this test should be invalidated
        if err:
            config.thApi.error_post(Errorparams(error="other-error",
                                         message="failed to sequester logs"))


def save_ps(src):
    with open(os.path.expanduser("~/logs/ps_%s_%s.log") % (src, datetime.datetime.now()), "w") as outfile:
        subprocess.call(["ps", "aux"], stdout=outfile)


def send_status(src, code, sendxy=True, sendtime=True):
    # optional in the API def and only send them if the robot's
    # been started, also sending time is optional
    try:
        x = -1.0
        y = -1.0
        if sendxy:
            x, y, ig1, ig2 = config.bot_cont.gazebo.get_bot_state()

        config.logger.debug("sending status %s from %s" % (code, src))

        if sendtime:
            dd = Statusparams(status=code,
                              x=x,
                              y=y,
                              charge=config.battery,
                              sim_time=rospy.Time.now().secs)
            rospy.loginfo(dd)
            config.logger.debug("Done message is %s" % dd)
            response = config.thApi.status_post(Statusparams(status=code,
                                                             x=x,
                                                             y=y,
                                                             charge=config.battery,
                                                             sim_time=rospy.Time.now().secs))
        else:
            dd = Statusparams(status=code,
                             x=x,
                             y=y,
                             charge=0,
                             sim_time=0
                             )
            rospy.loginfo(dd)
            config.logger.debug("Done message is %s" % dd)
            response = config.thApi.status_post(Statusparams(status=code,
                                                             x=x,
                                                             y=y,
                                                             charge=0,
                                                             sim_time=0
                                                             ))

        config.logger.debug("response from TH to status: %s" % response)

    except Exception as e:
        config.logger.error("Got an error %s when sending done" % e)


def send_done(src, msg, outcome):
    try:
        save_ps("done")
        x, y, ig1, ig2 = config.bot_cont.gazebo.get_bot_state()
        config.logger.debug("sending done from %s" % src)

        # right before posting, copy out all the logs
        sequester()

        response = config.thApi.done_post(Doneparams(x=x,
                                                     y=y,
                                                     charge=config.battery,
                                                     sim_time=rospy.Time.now().secs,
                                                     tasks_finished=config.tasks_finished,
                                                     outcome=outcome,
                                                     message=msg))
        config.logger.debug("response from TH to done: %s" % response)

    except Exception as e:
        config.logger.error("Got an error %s when sending status" % e)
