## from __future__ import with_statement ## todo: bring this back when you downgrade to py2

import subprocess
import os
import datetime
import math
import swagger_server.config as config
from swagger_client.models.parameters_1 import Parameters1
from swagger_client.models.parameters_2 import Parameters2
import rospy

def save_ps(src):
    with open(os.path.expanduser("~/ps_%s_%s.log") % (src, datetime.datetime.now()), "w") as outfile:
        subprocess.call(["ps","aux"],stdout=outfile)

def send_done(src):
    try:
        config.logger.debug("sending done from %s" % src)
        save_ps("done")
        x , y , w , v = config.cp.gazebo.get_turtlebot_state()
        d = Parameters2(final_x = x,
                                                  final_y = y,
                                                  final_sim_time = rospy.Time.now().secs,
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
          response = config.thApi.status_post(Parameters1(status = code,
                                                        message = msg,
                                                        sim_time = rospy.Time.now().secs,
                                                        plan = config.plan,
                                                        config = config.nodes,
                                                        sensors = config.sensors
          ))
          config.logger.info("repsonse from TH to status: %s" % response)
      except Exception as e:
          config.logger.error("Got an error %s when sending status" %e)
