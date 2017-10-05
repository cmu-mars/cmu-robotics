# coding: utf-8

from __future__ import absolute_import
from .base_model_ import Model
from datetime import date, datetime
from typing import List, Dict
from ..util import deserialize_model


class Perturbation(Model):
    """
    NOTE: This class is auto generated by the swagger code generator program.
    Do not edit the class manually.
    """
    def __init__(self, kind: str=None):
        """
        Perturbation - a model defined in Swagger

        :param kind: The kind of this Perturbation.
        :type kind: str
        """
        self.swagger_types = {
            'kind': str
        }

        self.attribute_map = {
            'kind': 'kind'
        }

        self._kind = kind

    @classmethod
    def from_dict(cls, dikt) -> 'Perturbation':
        """
        Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The Perturbation of this Perturbation.
        :rtype: Perturbation
        """
        return deserialize_model(dikt, cls)

    @property
    def kind(self) -> str:
        """
        Gets the kind of this Perturbation.
        Used to discriminate between different kinds of perturbation.

        :return: The kind of this Perturbation.
        :rtype: str
        """
        return self._kind

    @kind.setter
    def kind(self, kind: str):
        """
        Sets the kind of this Perturbation.
        Used to discriminate between different kinds of perturbation.

        :param kind: The kind of this Perturbation.
        :type kind: str
        """
        if kind is None:
            raise ValueError("Invalid value for `kind`, must not be `None`")

        self._kind = kind
