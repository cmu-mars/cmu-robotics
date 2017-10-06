# coding: utf-8

from __future__ import absolute_import
from .base_model_ import Model
from datetime import date, datetime
from typing import List, Dict
from ..util import deserialize_model


class InlineResponse2002(Model):
    """
    NOTE: This class is auto generated by the swagger code generator program.
    Do not edit the class manually.
    """
    def __init__(self, sim_time: int=None):
        """
        InlineResponse2002 - a model defined in Swagger

        :param sim_time: The sim_time of this InlineResponse2002.
        :type sim_time: int
        """
        self.swagger_types = {
            'sim_time': int
        }

        self.attribute_map = {
            'sim_time': 'sim-time'
        }

        self._sim_time = sim_time

    @classmethod
    def from_dict(cls, dikt) -> 'InlineResponse2002':
        """
        Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The inline_response_200_2 of this InlineResponse2002.
        :rtype: InlineResponse2002
        """
        return deserialize_model(dikt, cls)

    @property
    def sim_time(self) -> int:
        """
        Gets the sim_time of this InlineResponse2002.
        the simulation time when the battery was set

        :return: The sim_time of this InlineResponse2002.
        :rtype: int
        """
        return self._sim_time

    @sim_time.setter
    def sim_time(self, sim_time: int):
        """
        Sets the sim_time of this InlineResponse2002.
        the simulation time when the battery was set

        :param sim_time: The sim_time of this InlineResponse2002.
        :type sim_time: int
        """
        if sim_time is None:
            raise ValueError("Invalid value for `sim_time`, must not be `None`")

        self._sim_time = sim_time

