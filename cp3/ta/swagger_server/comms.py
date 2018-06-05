## from __future__ import with_statement ## todo: bring this back when you downgrade to py2

import subprocess
import os
import datetime
import math
import swagger_server.config as config
from swagger_client.models.parameters_1 import Parameters1
from swagger_client.models.parameters_2 import Parameters2
from ig_action_msgs.msg import InstructionGraphActionFeedback
import rospy

def save_ps(src):
    with open(os.path.expanduser("~/logs/ps_%s_%s.log") % (src, datetime.datetime.now()), "w") as outfile:
        subprocess.call(["ps","aux"],stdout=outfile)

def send_done(src):
    ## todo: this might be a little aggressive; this will bring down the TA
    ## entirely, i believe
    if (config.time_at_start == None):
        config.logger.error("trying to send done without having a start time")
        raise Exception("trying to send done without having a start time")

    try:
        config.time_at_done = rospy.Time.now().secs
        config.logger.debug("sending done from %s" % src)
        save_ps("done")
        x , y , w , v = config.cp.gazebo.get_turtlebot_state()
        d = Parameters2(final_x = x,
                        final_y = y,
                        final_sim_time = (config.time_at_done - config.time_at_start) - config.time_spent_adapting,
                        final_charge = math.floor(config.battery),
                        collisions = config.collisions,
                        num_adaptations = config.adaptations,
                        final_utility = 0 ## todo placeholder value
        )
        config.logger.debug("Done message is %s" %d)
        response = config.thApi.done_post(d)
        config.logger.debug("response from done: %s" % response)
    except Exception as e:
        config.logger.error("Got an error %s when sending done" %e)

def send_status(src, code, msg):
      try:
          config.logger.info("sending status %s from %s" % (code,src))
          status = Parameters1(status = code,
                                                        message = msg,
                                                        sim_time = rospy.Time.now().secs,
                                                        plan = config.plan,
                                                        config = config.nodes,
                                                        sensors = config.sensors
          )
          config.logger.debug("%s" %status)
          response = config.thApi.status_post(status)
          config.logger.info("repsonse from TH to status: %s" % response)
      except Exception as e:
          config.logger.error("Got an error %s when sending status" %e)

def setup_adapted_listeners():
  if not hasattr(config,"ig_subscriber") or config.ig_subscriber is None:
    config.adapted_state = 1
    config.ig_subscriber = rospy.Subscriber("/ig_action_server/feedback", InstructionGraphActionFeedback, track_adapted_state)
    config.logger.debug("Waiting for adapted processing")

def track_adapted_state(feedback):
  if config.adapted_state > 0:
    if "Received new valid" in feedback.feedback.sequence:
      config.logger.debug("Got new IG -- wairing for Move")
      config.adapted_state = 2
    elif config.adapted_state == 2 and "MoveAbs" in feedback.feedback.sequence and "START" in feedback.feedback.sequence:
      config.adapted_state = 0
      config.logger.debug('Got move, now sending adapted')
      config.ig_subscriber.unregister()
      config.ig_subscriber = None
      send_status("internal status, adapted, track_adapted_state", "adapted", "DAS has finished adapting, robot has finished any reconfigurations")
