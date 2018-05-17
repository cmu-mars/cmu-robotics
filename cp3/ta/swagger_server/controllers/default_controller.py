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
import swagger_server.config ## todo: maybe an easier way to do this; i don't know

def send_done(src):
    swagger_server.config.logger.debug("sending done from %s" % src)
    x , y , w , v = swagger_server.config.cp.gazebo.get_turtlebot_state()
    response = swagger_server.config.thApi.done_post(Parameters2(final_x = x,
                                                  final_y = y,
                                                  final_sim_time = rospy.Time.now().secs,
                                                  final_charge = swagger_server.config.battery,
                                                  collisions = [], ## todo placeholder value
                                                  num_adaptations = swagger_server.config.adaptations,
                                                  final_utility = 0 ## todo placeholder value
                                                  ))
    swagger_server.config.logger.debug("response from done: %s" % response)

def send_status(src, code, msg):
        swagger_server.config.logger.debug("sending status %s from %s" % (code,src))
        response = swagger_server.config.thApi.status_post(Parameters1(status = code,
                                                                       message = msg,
                                                                       sim_time = rospy.Time.now().secs,
                                                                       plan = cp.instruction_server.get_path(ready_resp.start_loc,ready_resp.target_loc), ## todo check
                                                                       config = swagger_server.config.nodes,
                                                                       sensors = swagger_server.config.sensors
        ))
        swagger_server.config.logger.debug("repsonse from TH to status: %s" % response)

def internal_status_post(CP3InternalStatus):  # noqa: E501
    """internal_status_post

    reports any internal status that might be sent to the TA for internal bookeeping or forwarding to the TH # noqa: E501

    :param CP3InternalStatus:
    :type CP3InternalStatus: dict | bytes

    :rtype: None
    """
    if connexion.request.is_json:
        CP3InternalStatus = CP3InternalStatus.from_dict(connexion.request.get_json())  # noqa: E501

    swagger_server.config.logger.debug("TA internal status end point hit with status %s and message %s"
                                       % (CP3InternalStatus.status, CP3InternalStatus.message))

    if CP3InternalStatus.status == "RAINBOW_READY":
        send_status("internal status, rainbow ready",
                    "live",
                    "CP3 TA ready to recieve inital perturbs in adaptive case after getting RAINBOW_READY")
        if not swagger_server.config.use_adaptation:
            swagger_server.config.logger.debug("[WARN] internal status got a rainbow ready when not in the adaptive case")

    elif CP3InternalStatus.status == "MISSION_SUCCEEDED":
        swagger_server.config.logger.debug("TA internal status got MISSION_SUCCEEDED")
        if swagger_server.config.use_adaptation:
            send_done("internal status, mission succeeded")
        else:
            swagger_server.config.logger.debug("[WARN] TA internal status got MISSION_SUCCEEDED in non-adapting case")

    elif CP3InternalStatus.status == "MISSION_FAILED":
        if swagger_server.config.use_adaptation:
            send_done("internal status, mission failed")
        else:
            swagger_server.config.logger.debug("[WARN] TA internal status got MISSION_FAILED in non-adapting case")

    elif CP3InternalStatus.status == "ADAPTING":
        swagger_server.config.adaptations = swagger_server.config.adaptations + 1
        send_status("internal status, adapting",
                    "adapting",
                    "DAS is now adapting")

    elif CP3InternalStatus.status == "ADAPTED":
        send_status("internal status, adapted",
                    "adapted",
                    "DAS has now adapted")

    elif CP3InternalStatus.status == "ADAPTED_FAILED":
        send_status("internal status, adapted_failed",
                    "adapted",
                    "DAS has now adapted after a failure with message %s" % CP3InternalStatus.message)

    elif CP3InternalStatus.status == "FINAL_UTILITY":
        swagger_server.config.logger.debug("ignoring until RR3") ## todo

    elif CP3InternalStatus.status == "PLAN":
        plan = [ x.strip() for x in  ast.literal_eval(CP3InternalStatus.message) ]
        swagger_server.config.logger.debug("ignoring until RR3") ## todo

def observe_get():
    """
    observe_get
    observe some of the current state of the robot for visualization and invariant checking for perturbation end points. n.b. this information is to be used strictly in a passive way; it is not to be used for evaluation of the test at all.

    :rtype: InlineResponse2003
    """
    x , y , w , z= swagger_server.config.cp.gazebo.get_turtlebot_state()

    if x == None or y == None:
        return {} , 400 ## todo: api doesn't specify any payload here

    ret = InlineResponse2003()
    ret.x = x
    ret.y = y
    ret.battery = swagger_server.config.battery
    ret.sim_time = rospy.Time.now().secs
    ret.lights = swagger_server.config.cp.map_server.lights_on()
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

    if not swagger_server.config.cp.map_server.is_light(Parameters.id):
        ret = InlineResponse4001(rospy.Time.now().secs, "invalid light name")
        return ret , 400

    if swagger_server.config.cp.gazebo.enable_light(Parameters.id, Parameters.state):
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

    retval , message = swagger_server.config.cp.kill_node(Parameters.id)

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
        res = swagger_server.config.cp.gazebo.set_kinect_mode('on')
    elif Parameters.id == "kinect" and not Parameters.state:
        res = swagger_server.config.cp.gazebo.set_kinect_mode('off')

    elif Parameters.id == "lidar" and Parameters.state:
        res = swagger_server.config.cp.gazebo.set_lidar_mode('on')
    elif Parameters.id == "lidar" and not Parameters.state:
        res = swagger_server.config.cp.gazebo.set_lidar_mode('off')

    elif Parameters.id == "camera" and Parameters.state:
        res = swagger_server.config.cp.gazebo.set_kinect_mode('image-only')
    elif Parameters.id == "camera" and not Parameters.state:
        res = swagger_server.config.cp.gazebo.set_kinect_mode('off')

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
    if not swagger_server.config.started:
        def active_cb():
            """ callback for when the bot is made active """
            swagger_server.config.logger.debug("received notification that goal is active")

        def done_cb(terminal, result):
            """ callback for when the bot is at the target """
            if 'successfully' in result.sequence:
                swagger_server.config.logger.debug("received notification that the goal has been completed successfully")

            if not swagger_server.config.use_adaptation:
                send_done("done callback")

        result , msg = swagger_server.config.cp.do_instructions(swagger_server.config.cp.start,
                                                                swagger_server.config.cp.target,
                                                                False,
                                                                active_cb,
                                                                done_cb)
        swagger_server.config.started = True
        return {} , 200
    else:
        ret = InlineResponse400("/start called more than once")
        return ret , 400
