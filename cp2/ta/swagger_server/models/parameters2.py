# coding: utf-8

from __future__ import absolute_import
from swagger_server.models.perturbation import Perturbation
from .base_model_ import Model
from datetime import date, datetime
from typing import List, Dict
from ..util import deserialize_model


class Parameters2(Model):
    """
    NOTE: This class is auto generated by the swagger code generator program.
    Do not edit the class manually.
    """
    def __init__(self, perturbations: List[Perturbation]=None):
        """
        Parameters2 - a model defined in Swagger

        :param perturbations: The perturbations of this Parameters2.
        :type perturbations: List[Perturbation]
        """
        self.swagger_types = {
            'perturbations': List[Perturbation]
        }

        self.attribute_map = {
            'perturbations': 'perturbations'
        }

        self._perturbations = perturbations

    @classmethod
    def from_dict(cls, dikt) -> 'Parameters2':
        """
        Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The Parameters_2 of this Parameters2.
        :rtype: Parameters2
        """
        return deserialize_model(dikt, cls)

    @property
    def perturbations(self) -> List[Perturbation]:
        """
        Gets the perturbations of this Parameters2.
        A non-empty list of perturbations to apply to the codebase

        :return: The perturbations of this Parameters2.
        :rtype: List[Perturbation]
        """
        return self._perturbations

    @perturbations.setter
    def perturbations(self, perturbations: List[Perturbation]):
        """
        Sets the perturbations of this Parameters2.
        A non-empty list of perturbations to apply to the codebase

        :param perturbations: The perturbations of this Parameters2.
        :type perturbations: List[Perturbation]
        """
        if perturbations is None:
            raise ValueError("Invalid value for `perturbations`, must not be `None`")

        self._perturbations = perturbations

