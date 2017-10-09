import connexion
from swagger_server.models.inline_response200 import InlineResponse200
from swagger_server.models.inline_response2001 import InlineResponse2001
from swagger_server.models.inline_response400 import InlineResponse400
from swagger_server.models.inline_response2001_resourceconsumption import InlineResponse2001Resourceconsumption
from swagger_server.models.parameters0 import Parameters0
from swagger_server.models.parameters1 import Parameters1
from swagger_server.models.parameters2 import Parameters2
from swagger_server.models.source_line import SourceLine
from swagger_server.models.perturbation import Perturbation
from swagger_server.models.compilation_outcome import CompilationOutcome
from swagger_server.models.degradation import Degradation
from swagger_server.models.test_qo_s import TestQoS
from swagger_server.models.test_outcome import TestOutcome
from swagger_server.models.candidate_adaptation import CandidateAdaptation
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

    rc = InlineResponse2001Resourceconsumption()
    rc.num_attempts = 5
    rc.time_spent = 43.2

    co = CompilationOutcome()
    co.time_taken = 500.0
    co.successful = True

    d = Degradation()

    q = TestQoS()
    q.duration = {}
    q.proximity = {}
    q.collisions = {}

    to = TestOutcome()
    to.test_id = "a"
    to.time_taken = "509"
    to.timed_out = False
    to.crashed = False
    to.qos = q

    ca = CandidateAdaptation()
    ca.diff = "1c1\n< a\n---\n> b\n"
    ca.complilation_outcome = co
    ca.degradation = d
    ca.test_outcomes = [ to ]

    ret = InlineResponse2001()
    ret.stage = "awaiting-perturbation"
    ret.resource_consumption = rc
    ret.pareto_set = [ ca ]

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
    return {} , 200


def perturbations_get(Parameters):
    """
    perturbations_get
    Returns a list of possible perturbations of an (optionally) specified shape and complexity that can be performed at a given line in the program. This endpoint should be used to select a suitable (set of) perturbation(s) for a test scenario.
    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: InlineResponse200
    """
    if connexion.request.is_json:
        Parameters = Parameters0.from_dict(connexion.request.get_json())

    inner = Perturbation()
    inner.kind = "swap arguments"

    ret = InlineResponse200()
    ret.perturbations = [ inner ]
    return ret
