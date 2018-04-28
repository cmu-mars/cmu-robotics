# coding: utf-8

from __future__ import absolute_import
from datetime import date, datetime  # noqa: F401

from typing import List, Dict  # noqa: F401

from swagger_server.models.base_model_ import Model
from swagger_server import util


class InlineResponse200(Model):
    """NOTE: This class is auto generated by the swagger code generator program.

    Do not edit the class manually.
    """

    def __init__(self, obstacleid: str=None, sim_time: int=None):  # noqa: E501
        """InlineResponse200 - a model defined in Swagger

        :param obstacleid: The obstacleid of this InlineResponse200.  # noqa: E501
        :type obstacleid: str
        :param sim_time: The sim_time of this InlineResponse200.  # noqa: E501
        :type sim_time: int
        """
        self.swagger_types = {
            'obstacleid': str,
            'sim_time': int
        }

        self.attribute_map = {
            'obstacleid': 'obstacleid',
            'sim_time': 'sim-time'
        }

        self._obstacleid = obstacleid
        self._sim_time = sim_time

    @classmethod
    def from_dict(cls, dikt) -> 'InlineResponse200':
        """Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The inline_response_200 of this InlineResponse200.  # noqa: E501
        :rtype: InlineResponse200
        """
        return util.deserialize_model(dikt, cls)

    @property
    def obstacleid(self) -> str:
        """Gets the obstacleid of this InlineResponse200.

        a unique identifier for this particular placed obstacle, so that it can be removed in the future  # noqa: E501

        :return: The obstacleid of this InlineResponse200.
        :rtype: str
        """
        return self._obstacleid

    @obstacleid.setter
    def obstacleid(self, obstacleid: str):
        """Sets the obstacleid of this InlineResponse200.

        a unique identifier for this particular placed obstacle, so that it can be removed in the future  # noqa: E501

        :param obstacleid: The obstacleid of this InlineResponse200.
        :type obstacleid: str
        """
        if obstacleid is None:
            raise ValueError("Invalid value for `obstacleid`, must not be `None`")  # noqa: E501

        self._obstacleid = obstacleid

    @property
    def sim_time(self) -> int:
        """Gets the sim_time of this InlineResponse200.

        the simulation time when the obstacle was placed  # noqa: E501

        :return: The sim_time of this InlineResponse200.
        :rtype: int
        """
        return self._sim_time

    @sim_time.setter
    def sim_time(self, sim_time: int):
        """Sets the sim_time of this InlineResponse200.

        the simulation time when the obstacle was placed  # noqa: E501

        :param sim_time: The sim_time of this InlineResponse200.
        :type sim_time: int
        """
        if sim_time is None:
            raise ValueError("Invalid value for `sim_time`, must not be `None`")  # noqa: E501

        self._sim_time = sim_time
