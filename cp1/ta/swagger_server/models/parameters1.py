# coding: utf-8

from __future__ import absolute_import
from .base_model_ import Model
from datetime import date, datetime
from typing import List, Dict
from ..util import deserialize_model


class Parameters1(Model):
    """
    NOTE: This class is auto generated by the swagger code generator program.
    Do not edit the class manually.
    """
    def __init__(self, obstacleid: str=None):
        """
        Parameters1 - a model defined in Swagger

        :param obstacleid: The obstacleid of this Parameters1.
        :type obstacleid: str
        """
        self.swagger_types = {
            'obstacleid': str
        }

        self.attribute_map = {
            'obstacleid': 'obstacleid'
        }

        self._obstacleid = obstacleid

    @classmethod
    def from_dict(cls, dikt) -> 'Parameters1':
        """
        Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The Parameters_1 of this Parameters1.
        :rtype: Parameters1
        """
        return deserialize_model(dikt, cls)

    @property
    def obstacleid(self) -> str:
        """
        Gets the obstacleid of this Parameters1.
        the obstacle ID given by /perturb/place-obstacle of the obstacle to be removed.

        :return: The obstacleid of this Parameters1.
        :rtype: str
        """
        return self._obstacleid

    @obstacleid.setter
    def obstacleid(self, obstacleid: str):
        """
        Sets the obstacleid of this Parameters1.
        the obstacle ID given by /perturb/place-obstacle of the obstacle to be removed.

        :param obstacleid: The obstacleid of this Parameters1.
        :type obstacleid: str
        """
        if obstacleid is None:
            raise ValueError("Invalid value for `obstacleid`, must not be `None`")

        self._obstacleid = obstacleid

