import connexion
from swagger_server.models.inline_response200 import InlineResponse200
from swagger_server.models.inline_response2001 import InlineResponse2001
from swagger_server.models.inline_response2002 import InlineResponse2002
from swagger_server.models.inline_response2003 import InlineResponse2003
from swagger_server.models.inline_response400 import InlineResponse400
from swagger_server.models.inline_response4001 import InlineResponse4001
from swagger_server.models.inline_response4002 import InlineResponse4002
from swagger_server.models.parameters0 import Parameters0
from swagger_server.models.parameters1 import Parameters1
from swagger_server.models.parameters2 import Parameters2
from datetime import date, datetime
from typing import List, Dict
from six import iteritems
from ..util import deserialize_date, deserialize_datetime

from swagger_server.models.cp3_internal_status import CP3InternalStatus  # noqa: E501

import ast
import rospy

import swagger_server.config as config
import swagger_server.comms as comm


def internal_status_post(CP3InternalStatus):  # noqa: E501
    """internal_status_post

    reports any internal status that might be sent to the TA for internal bookeeping or forwarding to the TH # noqa: E501

    :param CP3InternalStatus:
    :type CP3InternalStatus: dict | bytes

    :rtype: None
    """
    if connexion.request.is_json:
        CP3InternalStatus = CP3InternalStatus.from_dict(connexion.request.get_json())  # noqa: E501

    config.logger.debug("TA internal status end point hit with status %s and message %s"
                        % (CP3InternalStatus.status, CP3InternalStatus.message))

    if CP3InternalStatus.status == "RAINBOW_READY":
        comms.send_status("internal status, rainbow ready",
                          "live",
                          "CP3 TA ready to recieve inital perturbs in adaptive case after getting RAINBOW_READY")
        if not config.use_adaptation:
            config.logger.debug("[WARN] internal status got a rainbow ready when not in the adaptive case")

    elif CP3InternalStatus.status == "MISSION_SUCCEEDED":
        config.logger.debug("TA internal status got MISSION_SUCCEEDED")
        if config.use_adaptation:
            comms.send_done("internal status, mission succeeded")
        else:
            config.logger.debug("[WARN] TA internal status got MISSION_SUCCEEDED in non-adapting case")

    elif CP3InternalStatus.status == "MISSION_FAILED":
        if config.use_adaptation:
            comms.send_done("internal status, mission failed")
        else:
            config.logger.debug("[WARN] TA internal status got MISSION_FAILED in non-adapting case")

    elif CP3InternalStatus.status == "ADAPTING":
        config.adaptations = config.adaptations + 1
        comms.send_status("internal status, adapting",
                    "adapting",
                    "DAS is now adapting")

    elif CP3InternalStatus.status == "ADAPTED":
        comms.send_status("internal status, adapted",
                          "adapted",
                          "DAS has now adapted")

    elif CP3InternalStatus.status == "ADAPTED_FAILED":
        comms.send_status("internal status, adapted_failed",
                          "adapted",
                          "DAS has now adapted after a failure with message %s" % CP3InternalStatus.message)

    elif CP3InternalStatus.status == "FINAL_UTILITY":
        config.logger.debug("ignoring until RR3") ## todo

    elif CP3InternalStatus.status == "PLAN":
        plan = [ x.strip() for x in  ast.literal_eval(CP3InternalStatus.message) ]
        config.logger.debug("ignoring until RR3") ## todo

def observe_get():
    """
    observe_get
    observe some of the current state of the robot for visualization and invariant checking for perturbation end points. n.b. this information is to be used strictly in a passive way; it is not to be used for evaluation of the test at all.

    :rtype: InlineResponse2003
    """
    x , y , w , z = config.cp.gazebo.get_turtlebot_state()

    if x == None or y == None:
        return {} , 400 ## todo: api doesn't specify any payload here

    ret = InlineResponse2003()
    ret.x = x
    ret.y = y
    ret.battery = config.battery
    ret.sim_time = rospy.Time.now().secs
    ret.lights = config.cp.map_server.lights_on()
    return ret

def perturb_light_post(Parameters):
    """
    perturb_light_post
    change the state of a light in the environment
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse200
    """
    if connexion.request.is_json:
        Parameters = Parameters0.from_dict(connexion.request.get_json())

    if not config.cp.map_server.is_light(Parameters.id):
        ret = InlineResponse4001(rospy.Time.now().secs, "invalid light name")
        return ret , 400

    if config.cp.gazebo.enable_light(Parameters.id, Parameters.state):
        ret = InlineResponse200()
        ret.sim_time = rospy.Time.now().secs
        return ret
    else:
        ret = InlineResponse4001(rospy.Time.now().secs, "light setting failed on: %s" % Parameters)
        return ret , 400

def perturb_nodefail_post(Parameters):
    """
    perturb_nodefail_post
    cause one of the software nodes to fail
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse2002
    """
    if connexion.request.is_json:
        Parameters = Parameters2.from_dict(connexion.request.get_json())

    retval , message = config.cp.kill_node(Parameters.id)

    if retval:
        ret = InlineResponse2002()
        ret.sim_time = rospy.Time.now().secs
        return ret
    else:
        ret = InlineResponse4001()
        ret.sim_time = rospy.Time.now().secs
        ret.message = message
        return ret , 400

def perturb_sensor_post(Parameters):
    """
    perturb_sensor_post
    change the state of one of the sensors on the robot
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse2001
    """
    if connexion.request.is_json:
        Parameters = Parameters1.from_dict(connexion.request.get_json())

    res = None
    if Parameters.id == "kinect" and Parameters.state:
        res = config.cp.gazebo.set_kinect_mode('on')
    elif Parameters.id == "kinect" and not Parameters.state:
        res = config.cp.gazebo.set_kinect_mode('off')

    elif Parameters.id == "lidar" and Parameters.state:
        res = config.cp.gazebo.set_lidar_mode('on')
    elif Parameters.id == "lidar" and not Parameters.state:
        res = config.cp.gazebo.set_lidar_mode('off')

    elif Parameters.id == "camera" and Parameters.state:
        res = config.cp.gazebo.set_kinect_mode('image-only')
    elif Parameters.id == "camera" and not Parameters.state:
        res = config.cp.gazebo.set_kinect_mode('off')

    if res:
        ret = InlineResponse2001()
        ret.sim_time = rospy.Time.now().secs
        return ret
    else:
        ret = InlineResponse4001()
        ret.sim_time = rospy.Time.now.secs()
        ret.message = ("failed to set sensor state: %s" % Parameters)
        return ret , 400

def start_post():
    """
    start_post
    start the turtlebot navigating through the map

    :rtype: None
    """
    if not config.started:
        def active_cb():
            """ callback for when the bot is made active """
            config.logger.debug("received notification that goal is active")

        def done_cb(terminal, result):
            """ callback for when the bot is at the target """
            if 'successfully' in result.sequence:
                config.logger.debug("received notification that the goal has been completed successfully")

            if not config.use_adaptation:
                comms.send_done("done callback")

        result , msg = config.cp.do_instructions(config.cp.start,
                                                 config.cp.target,
                                                 False,
                                                 active_cb,
                                                 done_cb)
        config.started = True
        return {} , 200
    else:
        ret = InlineResponse400("/start called more than once")
        return ret , 400
