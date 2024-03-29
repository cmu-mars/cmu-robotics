# coding: utf-8

from __future__ import absolute_import
from .base_model_ import Model
from datetime import date, datetime
from typing import List, Dict
from ..util import deserialize_model


class Parameters0(Model):
    """
    NOTE: This class is auto generated by the swagger code generator program.
    Do not edit the class manually.
    """
    def __init__(self, id: str=None, state: bool=None):
        """
        Parameters - a model defined in Swagger

        :param id: The id of this Parameters.
        :type id: str
        :param state: The state of this Parameters.
        :type state: bool
        """
        self.swagger_types = {
            'id': str,
            'state': bool
        }

        self.attribute_map = {
            'id': 'id',
            'state': 'state'
        }

        self._id = id
        self._state = state

    @classmethod
    def from_dict(cls, dikt) -> 'Parameters':
        """
        Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The Parameters of this Parameters.
        :rtype: Parameters
        """
        return deserialize_model(dikt, cls)

    @property
    def id(self) -> str:
        """
        Gets the id of this Parameters.
        the element of LIGHTSET whose state will be set

        :return: The id of this Parameters.
        :rtype: str
        """
        return self._id

    @id.setter
    def id(self, id: str):
        """
        Sets the id of this Parameters.
        the element of LIGHTSET whose state will be set

        :param id: The id of this Parameters.
        :type id: str
        """
        if id is None:
            raise ValueError("Invalid value for `id`, must not be `None`")

        self._id = id

    @property
    def state(self) -> bool:
        """
        Gets the state of this Parameters.
        upon response, the named light will be on if `true` and `off` otherwise

        :return: The state of this Parameters.
        :rtype: bool
        """
        return self._state

    @state.setter
    def state(self, state: bool):
        """
        Sets the state of this Parameters.
        upon response, the named light will be on if `true` and `off` otherwise

        :param state: The state of this Parameters.
        :type state: bool
        """
        if state is None:
            raise ValueError("Invalid value for `state`, must not be `None`")

        self._state = state
