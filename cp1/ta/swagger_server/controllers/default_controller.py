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

     # todo -- mirror cp3, roughly

    return 'do some magic!'

def observe_get():
    """
    observe_get
    observe some of the current state of the robot for visualization and invariant checking for perturbation end points. n.b. this information is to be used strictly in a passive way; it is not to be used for evaluation of the test at all.

    :rtype: InlineResponse2003
    """

    ret = InlineResponse2003()
    ret.x = 0.0 #todo
    ret.y = 0.0 #todo
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

    # todo actually set the battery

    ret = InlineResponse2002()
    ret.sim_time = rospy.Time.now().secs
    return ret


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

    # todo: dynamic check goes here against parameter x and y? maybe
    # the obstacle will just fail to place

    # todo place obstacle

    ret = InlineResponse200()
    ret.obstacleid = "obs1" # todo get names
    ret.sim_time = rospy.Time.now().secs
    return ret


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

    # this is a bit of a hack for RR1 -- this is the only name we ever give
    # out so it's the only one we can accept
    if not Parameters.obstacleid == "obs1":
        ret = InlineReponse4001()
        ret.cause = "bad-obstacleid"
        ret.message = "got an obstacle ID other than 'obs1'"
        return ret , 400

    ret = InlineResponse2001()
    ret.sim_time = rospy.Time.now().secs
    return ret


def start_post():
    """
    start_post
    start the turtlebot on the mission

    :rtype: None
    """

    #todo

    return {}
