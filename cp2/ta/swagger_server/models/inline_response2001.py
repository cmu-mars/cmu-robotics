# coding: utf-8

from __future__ import absolute_import
from datetime import date, datetime  # noqa: F401

from typing import List, Dict  # noqa: F401

from swagger_server.models.base_model_ import Model
from swagger_server.models.candidate_adaptation import CandidateAdaptation  # noqa: F401,E501
from swagger_server.models.inline_response2001_resourceconsumption import InlineResponse2001Resourceconsumption  # noqa: F401,E501
from swagger_server import util


class InlineResponse2001(Model):
    """NOTE: This class is auto generated by the swagger code generator program.

    Do not edit the class manually.
    """

    def __init__(self, stage: str=None, resource_consumption: InlineResponse2001Resourceconsumption=None, pareto_set: List[CandidateAdaptation]=None):  # noqa: E501
        """InlineResponse2001 - a model defined in Swagger

        :param stage: The stage of this InlineResponse2001.  # noqa: E501
        :type stage: str
        :param resource_consumption: The resource_consumption of this InlineResponse2001.  # noqa: E501
        :type resource_consumption: InlineResponse2001Resourceconsumption
        :param pareto_set: The pareto_set of this InlineResponse2001.  # noqa: E501
        :type pareto_set: List[CandidateAdaptation]
        """
        self.swagger_types = {
            'stage': str,
            'resource_consumption': InlineResponse2001Resourceconsumption,
            'pareto_set': List[CandidateAdaptation]
        }

        self.attribute_map = {
            'stage': 'stage',
            'resource_consumption': 'resource-consumption',
            'pareto_set': 'pareto-set'
        }

        self._stage = stage
        self._resource_consumption = resource_consumption
        self._pareto_set = pareto_set

    @classmethod
    def from_dict(cls, dikt) -> 'InlineResponse2001':
        """Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The inline_response_200_1 of this InlineResponse2001.  # noqa: E501
        :rtype: InlineResponse2001
        """
        return util.deserialize_model(dikt, cls)

    @property
    def stage(self) -> str:
        """Gets the stage of this InlineResponse2001.

        A concise description of the current state of the system.  # noqa: E501

        :return: The stage of this InlineResponse2001.
        :rtype: str
        """
        return self._stage

    @stage.setter
    def stage(self, stage: str):
        """Sets the stage of this InlineResponse2001.

        A concise description of the current state of the system.  # noqa: E501

        :param stage: The stage of this InlineResponse2001.
        :type stage: str
        """
        allowed_values = ["READY_TO_PERTURB", "PERTURBING", "READY_TO_ADAPT", "SEARCHING", "FINISHED"]  # noqa: E501
        if stage not in allowed_values:
            raise ValueError(
                "Invalid value for `stage` ({0}), must be one of {1}"
                .format(stage, allowed_values)
            )

        self._stage = stage

    @property
    def resource_consumption(self) -> InlineResponse2001Resourceconsumption:
        """Gets the resource_consumption of this InlineResponse2001.


        :return: The resource_consumption of this InlineResponse2001.
        :rtype: InlineResponse2001Resourceconsumption
        """
        return self._resource_consumption

    @resource_consumption.setter
    def resource_consumption(self, resource_consumption: InlineResponse2001Resourceconsumption):
        """Sets the resource_consumption of this InlineResponse2001.


        :param resource_consumption: The resource_consumption of this InlineResponse2001.
        :type resource_consumption: InlineResponse2001Resourceconsumption
        """

        self._resource_consumption = resource_consumption

    @property
    def pareto_set(self) -> List[CandidateAdaptation]:
        """Gets the pareto_set of this InlineResponse2001.

        A list containing details of the sub-set of adaptations that have been encountered that belong to the pareto set (i.e., the set of non-dominated adaptations).  # noqa: E501

        :return: The pareto_set of this InlineResponse2001.
        :rtype: List[CandidateAdaptation]
        """
        return self._pareto_set

    @pareto_set.setter
    def pareto_set(self, pareto_set: List[CandidateAdaptation]):
        """Sets the pareto_set of this InlineResponse2001.

        A list containing details of the sub-set of adaptations that have been encountered that belong to the pareto set (i.e., the set of non-dominated adaptations).  # noqa: E501

        :param pareto_set: The pareto_set of this InlineResponse2001.
        :type pareto_set: List[CandidateAdaptation]
        """

        self._pareto_set = pareto_set
