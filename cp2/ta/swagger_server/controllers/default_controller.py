import logging

import connexion
import six

from swagger_server.models.error import Error  # noqa: E501
from swagger_server.models.inline_response200 import InlineResponse200  # noqa: E501
from swagger_server.models.inline_response2001 import InlineResponse2001  # noqa: E501
from swagger_server.models.inline_response2001_resourceconsumption import InlineResponse2001Resourceconsumption  # noqa: E501
from swagger_server.models.parameters import Parameters  # noqa: E501
from swagger_server.models.perturbation import Perturbation  # noqa: E501
from swagger_server.models.perturbation_params import PerturbationParams  # noqa: E501
from swagger_server.models.source_line import SourceLine  # noqa: E501
from swagger_server import util
import swagger_server.models

from orchestrator import Orchestrator
from orchestrator.exceptions import *

from swagger_server import config
from swagger_server.converters import *

logger = logging.getLogger('cp2ta').getChild('controller')  # type: logging.Logger
logger.setLevel(logging.DEBUG)


def adapt_post(Parameters):  # noqa: E501
    """adapt_post

    Triggers the code adaptation process. # noqa: E501

    :param Parameters:
    :type Parameters: dict | bytes

    :rtype: None
    """
    logger.debug("passed Parameters: %s", Parameters)
    if connexion.request.is_json:
        jsn = connexion.request.get_json()
        logger.debug("payload: %s", jsn)
        Parameters = swagger_server.models.parameters.Parameters.from_dict(jsn)  # noqa: E501
        logger.debug("parameters: %s", Parameters)
    else:
        logger.warning("apparently the request is not JSON")

    logger.debug("triggering adaptation")
    if Parameters.time_limit is not None:
        logger.info("* using time limit of %d minutes", Parameters.time_limit)
    if Parameters.attempt_limit is not None:
        logger.info("* using attempt limit of %d attempts", Parameters.attempt_limit)

    try:
        config.orc.adapt(minutes=Parameters.time_limit,
                         attempts=Parameters.attempt_limit)
        logger.debug("triggered adaptation")
        return '', 202
    except OrchestratorError as err:
        logger.exception("failed to trigger adaptation: %s", err)
        return err.to_response()


def files_get():  # noqa: E501
    """files_get

    Returns a list of all the source files that may be subject to perturbation. # noqa: E501


    :rtype: List[str]
    """
    return config.orc.files


def lines_get():  # noqa: E501
    """lines_get

    Returns a list of all the source lines at which perturbations may be injected. # noqa: E501


    :rtype: List[SourceLine]
    """
    return [SourceLine(line.filename, line.num) for line in config.orc.lines]

def observe_get():  # noqa: E501
    """observe_get

    Returns the current status of the SUT. # noqa: E501


    :rtype: InlineResponse2001
    """
    num_attempts, time_spent = config.orc.resource_usage
    jsn_patches = config.orc.patches

    resources = InlineResponse2001Resourceconsumption()
    resources.num_attempts = num_attempts
    resources.time_spent = time_spent

    ret = InlineResponse2001()
    ret.stage = config.orc.state.name
    ret.resource_consumption = resources
    ret.pareto_set = [ patch2ca(patch) for patch in config.orc.patches ]

    return ret

def perturb_post(perturb_params):  # noqa: E501
    """perturb_post

    Applies a given perturbation to the SUT. The resulting perturbed system will become Baseline B, provided that the system builds and fails at least one test. # noqa: E501

    :param perturb_params:
    :type perturb_params: dict | bytes

    :rtype: None
    """
    logger.debug("Attempting to perturb system.")
    if connexion.request.is_json:
        jsn = connexion.request.get_json()
        logger.debug("Decoding perturbation from provided dict: %s", jsn)
        perturb_params = Perturbation.from_dict(jsn)
        logger.debug("Decoded perturbation from provided dict.")

    logger.debug("Transforming perturbation into a boggart mutation.")
    mutation = perturb2mutation(perturb_params)
    logger.debug("Transformed perturbation into a boggart mutation.")
    try:
        config.orc.perturb(mutation)
        logger.info("Successfully perturbed system using mutation: %s",
                    mutation)
        return '', 204
    except OrchestratorError as err:
        logger.exception("Failed to apply given mutation: %s",
                         mutation)
        return err.to_response()

def perturbations_get(perturbation_params):  # noqa: E501
    """perturbations_get

    Returns a list of possible perturbations of an (optionally) specified shape and complexity that can be performed at a given line in the program. This endpoint should be used to select a suitable (set of) perturbation(s) for a test scenario. # noqa: E501

    :param perturbation_params:
    :type perturbation_params: dict | bytes

    :rtype: InlineResponse200
    """
    if connexion.request.is_json:
        pp = PerturbationParams.from_dict(connexion.request.get_json())  # noqa: E501

    mutations = config.orc.perturbations(filename=pp.file,
                                           line_num=pp.line,
                                           op_name=pp.shape)

    return InlineResponse200([mutation2perturb(m) for m in mutations])
