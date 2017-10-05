import connexion
from swagger_server.models.inline_response200 import InlineResponse200
from swagger_server.models.inline_response2001 import InlineResponse2001
from swagger_server.models.inline_response2002 import InlineResponse2002
from swagger_server.models.inline_response400 import InlineResponse400
from swagger_server.models.inline_response4001 import InlineResponse4001
from swagger_server.models.inline_response4002 import InlineResponse4002
from swagger_server.models.inline_response4003 import InlineResponse4003
from swagger_server.models.parameters import Parameters
from swagger_server.models.parameters1 import Parameters1
from swagger_server.models.parameters2 import Parameters2
from datetime import date, datetime
from typing import List, Dict
from six import iteritems
from ..util import deserialize_date, deserialize_datetime


def action_start_post():
    """
    action_start_post
    start the turtlebot on the mission

    :rtype: None
    """
    return 'do some magic!'


def perturb_battery_post(Parameters=None):
    """
    perturb_battery_post
    set the level of the battery in a currently running test
    :param Parameters: 
    :type Parameters: dict | bytes

    :rtype: InlineResponse2002
    """
    if connexion.request.is_json:
        Parameters = Parameters2.from_dict(connexion.request.get_json())
    return 'do some magic!'


def perturb_place_obstacle_post(Parameters=None):
    """
    perturb_place_obstacle_post
    if the test is running, then place an instance of the obstacle on the map
    :param Parameters: 
    :type Parameters: dict | bytes

    :rtype: InlineResponse200
    """
    if connexion.request.is_json:
        Parameters = Parameters.from_dict(connexion.request.get_json())
    return 'do some magic!'


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
    return 'do some magic!'
