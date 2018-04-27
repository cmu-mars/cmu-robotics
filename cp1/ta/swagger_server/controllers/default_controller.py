import connexion
import six

from swagger_server.models.battery_params import BatteryParams  # noqa: E501
from swagger_server.models.cp1_internal_status import CP1InternalStatus  # noqa: E501
from swagger_server.models.inline_response200 import InlineResponse200  # noqa: E501
from swagger_server.models.inline_response2001 import InlineResponse2001  # noqa: E501
from swagger_server.models.inline_response2002 import InlineResponse2002  # noqa: E501
from swagger_server.models.inline_response2003 import InlineResponse2003  # noqa: E501
from swagger_server.models.inline_response400 import InlineResponse400  # noqa: E501
from swagger_server.models.inline_response4001 import InlineResponse4001  # noqa: E501
from swagger_server.models.inline_response4002 import InlineResponse4002  # noqa: E501
from swagger_server.models.inline_response4003 import InlineResponse4003  # noqa: E501
from swagger_server.models.place_params import PlaceParams  # noqa: E501
from swagger_server.models.remove_params import RemoveParams  # noqa: E501
from swagger_server import util

## todo remove?
from datetime import date, datetime
from typing import List, Dict
from six import iteritems
from ..util import deserialize_date, deserialize_datetime

import swagger_server.config as config

import rospy

def internal_post(CP1InternalStatus):  # noqa: E501
    """internal_post

    reports any internal status (including the error that may occured) from the backend that might be sent to the TA for internal bookeeping or forwarding to the TH # noqa: E501

    :param CP1InternalStatus:
    :type CP1InternalStatus: dict | bytes

    :rtype: None
    """
    if connexion.request.is_json:
        CP1InternalStatus = CP1InternalStatus.from_dict(connexion.request.get_json())  # noqa: E501

    config.logger.debug("TA internal status end point hit with status %s and message %s"
                        % (CP1InternalStatus.status, CP1InternalStatus.message))

    if CP1InternalStatus.status == "learning-started":
        config.logger.debug("internal got a deprecated status which is being ignored")
        pass
    elif CP1InternalStatus.status == "learning-done":
        config.logger.debug("internal got a deprecated status which is being ignored")
        pass
    elif CP1InternalStatus.status == "adapt-started":
        ## send to TH; does this ever happen or only from rainbow as below?
    elif CP1InternalStatus.status == "adapt-done":
        ## send to TH; does this ever happen or only from rainbow as below?
    elif CP1InternalStatus.status == "charging-started":
        ## send to TH
    elif CP1InternalStatus.status == "charging-done":
        ## send to TH
    elif CP1InternalStatus.status == "parsing-error":
        config.logger.debug("internal got a deprecated status which is being ignored")
        pass
    elif CP1InternalStatus.status == "learning-error":
        config.logger.debug("internal got a deprecated status which is being ignored")
        pass
    elif CP1InternalStatus.status ==  "other-error":
        ## send error to the TH

    ## these are the literal constants that come from rainbow. the
    ## constants above are from the API definition; there's some
    ## overlap and this is a little messy
    elif CP1InternalStatus.status == "RAINBOW_READY":
        ## send live, log if not in C
    elif CP1InternalStatus.status == "MISSION_SUCCEEDED":
        ## not clear because of mission sequencer
    elif CP1InternalStatus.status == "MISSION_FAILED":
        ## not clear because of mission sequencer
    elif CP1InternalStatus.status == "ADAPTING":
        ## send adapt-started
    elif CP1InternalStatus.status == "ADAPTED":
        ## send adapt-done
    elif CP1InternalStatus.status == "ADAPTED_FAILED":
        ## not clear

def observe_get():
    """
    observe_get
    observe some of the current state of the robot for visualization and invariant checking for perturbation end points. n.b. this information is to be used strictly in a passive way; it is not to be used for evaluation of the test at all.

    :rtype: InlineResponse2003
    """
    x , y , ig1 , ig2 = config.bot_cont.gazebo.get_bot_state()

    ret = InlineResponse2003()
    ret.x = x
    ret.y = y
    ret.battery = config.battery
    ret.sim_time = rospy.Time.now().secs

    return ret


def perturb_battery_post(Parameters=None):
    """
    perturb_battery_post
    set the level of the battery in a currently running test. consistent with the monotonicity requirement for the power model, this cannot be more than the current amount of charge in the battery.
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse2002
    """
    if connexion.request.is_json:
        BatteryParams = BatteryParams.from_dict(connexion.request.get_json())  # noqa: E501

    if config.bot_cont.gazebo.set_charge(BatteryParams.charge):
        return InlineResponse2002(sim_time = rospy.Time.now().secs)
    else:
        return InlineResponse4002(message = "setting the battery failed") , 400


def perturb_place_obstacle_post(Parameters=None):
    """
    perturb_place_obstacle_post
    if the test is running, then place an instance of the obstacle on the map
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse200
    """
    if connexion.request.is_json:
        PlaceParams = PlaceParams.from_dict(connexion.request.get_json())  # noqa: E501

    result = config.bot_cont.gazebo.place_obstacle(PlaceParams.x, PlaceParams.y)
    if result:
        return InlineResponse200(obstacleid = result, sim_time = rospy.Time.now().secs)
    else:
        ## todo: we can't really distinguish between reasons for
        ## failure here so the API is a little bit too big
        return InlineResponse4001(cause = "other-error", message="obstacle placement failed")

def perturb_remove_obstacle_post(Parameters=None):
    """
    perturb_remove_obstacle_post
    if the test is running, remove a previously placed obstacle from the map
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse2001
    """
    if connexion.request.is_json:
        RemoveParams = RemoveParams.from_dict(connexion.request.get_json())  # noqa: E501

    if config.bot_cont.gazebo.remove_obstacle(RemoveParams.obstacleid):
        return InlineResponse2001(sim_time=rospy.Time.now().secs)
    else:
        return InlineReponse4001(cause="bad-obstacleid",
                                 message="asked to remove an obstacle with a name we didn't issue")

def start_post():
    """
    start_post
    start the turtlebot on the mission

    :rtype: None
    """

    if config.level == "c":
        config.bot_cont.go_instructions_multiple_tasks_adaptive( , )
    else:


    return {}
