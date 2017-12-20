import connexion
from swagger_server.models.inline_response200 import InlineResponse200
from swagger_server.models.inline_response2001 import InlineResponse2001
from swagger_server.models.inline_response2002 import InlineResponse2002
from swagger_server.models.inline_response2003 import InlineResponse2003
from swagger_server.models.inline_response400 import InlineResponse400
from swagger_server.models.inline_response4001 import InlineResponse4001
from swagger_server.models.inline_response4002 import InlineResponse4002
from swagger_server.models.inline_response4003 import InlineResponse4003
from swagger_server.models.parameters0 import Parameters0
from swagger_server.models.parameters1 import Parameters1
from swagger_server.models.parameters2 import Parameters2
from datetime import date, datetime
from typing import List, Dict
from six import iteritems
from ..util import deserialize_date, deserialize_datetime

import swagger_server.config as config

def observe_get():
    """
    observe_get
    observe some of the current state of the robot for visualization and invariant checking for perturbation end points. n.b. this information is to be used strictly in a passive way; it is not to be used for evaluation of the test at all.

    :rtype: InlineResponse2003
    """

    ret = InlineResponse2003()
    ret.x = 0.0
    ret.y = 0.0
    ret.battery = 5
    ret.sim_time = 5

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
        Parameters = Parameters2.from_dict(connexion.request.get_json())

    if Parameters.charge > config.ready_response.max_charge:
        ret = InlineResponse4002()
        ret.message = "battery cannot be set to a charge higher than the max charge given in the response to /ready"
        return ret , 400

    ret = InlineResponse2002()
    ret.sim_time = 0
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
        Parameters = Parameters0.from_dict(connexion.request.get_json())

    # todo: dynamic check goes here against parameter x and y

    ret = InlineResponse200()
    ret.obstacleid = "obs1"
    ret.topleft_x = 0.0
    ret.topleft_y = 0.0
    ret.botright_x = 0.0
    ret.botright_y = 0.0
    ret.sim_time = 0
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
        Parameters = Parameters1.from_dict(connexion.request.get_json())

    # this is a bit of a hack for RR1 -- this is the only name we ever give
    # out so it's the only one we can accept
    if not Parameters.obstacleid == "obs1":
        ret = InlineReponse4001()
        ret.cause = "bad-obstacleid"
        ret.message = "got an obstacle ID other than 'obs1'"
        return ret , 400

    ret = InlineResponse2001()
    ret.sim_time = 0
    return ret


def start_post():
    """
    start_post
    start the turtlebot on the mission

    :rtype: None
    """
    return {}
