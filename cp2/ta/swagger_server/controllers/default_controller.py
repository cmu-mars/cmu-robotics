import connexion
from swagger_server.models.inline_response200 import InlineResponse200
from swagger_server.models.inline_response400 import InlineResponse400
from swagger_server.models.parameters import Parameters
from swagger_server.models.parameters1 import Parameters1
from swagger_server.models.parameters2 import Parameters2
from datetime import date, datetime
from typing import List, Dict
from six import iteritems
from ..util import deserialize_date, deserialize_datetime


def adapt_post(Parameters):
    """
    adapt_post
    Used to trigger the code adaptation process.
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: None
    """
    if connexion.request.is_json:
        Parameters = Parameters1.from_dict(connexion.request.get_json())
    return 'do some magic!'


def observe_get():
    """
    observe_get
    Returns the current status of the SUT. Note / TODO: this will be either filled in or removed as we work on the verdict expression and determine what observations are relevant to this CP.

    :rtype: None
    """
    return 'do some magic!'


def perturb_post(Parameters):
    """
    perturb_post
    Applies a set of perturbations, given as a list of JSON objects, to the SUT. This endpoint should be used to prepare a test scenario for evaluation.
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: None
    """
    if connexion.request.is_json:
        Parameters = Parameters2.from_dict(connexion.request.get_json())
    return 'do some magic!'


def perturbations_get(Parameters):
    """
    perturbations_get
    Returns a list of possible perturbations of an (optionally) specified shape and complexity that can be performed at a given (set of) location(s) in the program.  This endpoint should be used to select a suitable (set of) perturbation(s) for a test scenario.
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse200
    """
    if connexion.request.is_json:
        Parameters = Parameters.from_dict(connexion.request.get_json())
    return 'test'
