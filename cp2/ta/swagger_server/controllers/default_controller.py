import connexion
from swagger_server.models.inline_response200 import InlineResponse200
from swagger_server.models.inline_response2001 import InlineResponse2001
from swagger_server.models.inline_response400 import InlineResponse400
from swagger_server.models.inline_response2001_resourceconsumption import InlineResponse2001Resourceconsumption
from swagger_server.models.parameters import Parameters
from swagger_server.models.parameters1 import Parameters1
from swagger_server.models.parameters2 import Parameters2
from swagger_server.models.source_line import SourceLine
from swagger_server.models.perturbation import Perturbation
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

    return {} , 200


def lines_get():
    """
    lines_get
    Returns a list of all the source lines at which perturbations may be injected.

    :rtype: List[SourceLine]
    """

    dummy_line = SourceLine()
    dummy_line.file = "main.cpp"
    dummy_line.number = 500

    return [ dummy_line ]


def observe_get():
    """
    observe_get
    Returns the current status of the SUT.

    :rtype: InlineResponse2001
    """

    inner = InlineResponse2001Resourceconsumption()
    inner.num_attempts = 5
    inner.time_spent = 43.2

    ret = InlineResponse2001()
    ret.stage = "awaiting-perturbation"
    ret.resource_consumption = inner
    ret.pareto_set = [ "fix it!" ]

    return ret


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
    Returns a list of possible perturbations of an (optionally) specified shape and complexity that can be performed at a given line in the program. This endpoint should be used to select a suitable (set of) perturbation(s) for a test scenario.
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse200
    """
    ## TODO
    # if connexion.request.is_json:
    #     Parameters = Parameters.from_dict(connexion.request.get_json())

    inner = Perturbation()
    inner.kind = "swap arguments"

    ret = InlineResponse200()
    ret.perturbations = [ inner ]
    return ret
