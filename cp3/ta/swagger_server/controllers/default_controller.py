import connexion
from swagger_server.models.inline_response200 import InlineResponse200
from swagger_server.models.inline_response2001 import InlineResponse2001
from swagger_server.models.inline_response2002 import InlineResponse2002
from swagger_server.models.inline_response2003 import InlineResponse2003
from swagger_server.models.inline_response400 import InlineResponse400
from swagger_server.models.inline_response4001 import InlineResponse4001
from swagger_server.models.inline_response4002 import InlineResponse4002
from swagger_server.models.parameters import Parameters
from swagger_server.models.parameters1 import Parameters1
from swagger_server.models.parameters2 import Parameters2
from datetime import date, datetime
from typing import List, Dict
from six import iteritems
from ..util import deserialize_date, deserialize_datetime


def action_observe_get():
    """
    action_observe_get
    observe some of the current state of the robot for visualization and invariant checking for perturbation end points. n.b. this information is to be used strictly in a passive way; it is not to be used for evaluation of the test at all.

    :rtype: InlineResponse2003
    """
    return 'do some magic!'


def perturb_light_post(Parameters):
    """
    perturb_light_post
    change the state of a light in the environment
    :param Parameters: 
    :type Parameters: dict | bytes

    :rtype: InlineResponse200
    """
    if connexion.request.is_json:
        Parameters = Parameters.from_dict(connexion.request.get_json())
    return 'do some magic!'


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
    return 'do some magic!'


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
    return 'do some magic!'


def start_post():
    """
    start_post
    start the turtlebot navigating through the map

    :rtype: None
    """
    return 'do some magic!'
