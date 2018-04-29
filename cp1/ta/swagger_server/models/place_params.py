# coding: utf-8

from __future__ import absolute_import
from datetime import date, datetime  # noqa: F401

from typing import List, Dict  # noqa: F401

from swagger_server.models.base_model_ import Model
from swagger_server import util


class PlaceParams(Model):
    """NOTE: This class is auto generated by the swagger code generator program.

    Do not edit the class manually.
    """

    def __init__(self, x: float=None, y: float=None):  # noqa: E501
        """PlaceParams - a model defined in Swagger

        :param x: The x of this PlaceParams.  # noqa: E501
        :type x: float
        :param y: The y of this PlaceParams.  # noqa: E501
        :type y: float
        """
        self.swagger_types = {
            'x': float,
            'y': float
        }

        self.attribute_map = {
            'x': 'x',
            'y': 'y'
        }

        self._x = x
        self._y = y

    @classmethod
    def from_dict(cls, dikt) -> 'PlaceParams':
        """Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The PlaceParams of this PlaceParams.  # noqa: E501
        :rtype: PlaceParams
        """
        return util.deserialize_model(dikt, cls)

    @property
    def x(self) -> float:
        """Gets the x of this PlaceParams.

        the x-coordinate of the center of the obstacle placement position  # noqa: E501

        :return: The x of this PlaceParams.
        :rtype: float
        """
        return self._x

    @x.setter
    def x(self, x: float):
        """Sets the x of this PlaceParams.

        the x-coordinate of the center of the obstacle placement position  # noqa: E501

        :param x: The x of this PlaceParams.
        :type x: float
        """
        if x is None:
            raise ValueError("Invalid value for `x`, must not be `None`")  # noqa: E501

        self._x = x

    @property
    def y(self) -> float:
        """Gets the y of this PlaceParams.

        the y-coordinate of the center of the obstacle placement position  # noqa: E501

        :return: The y of this PlaceParams.
        :rtype: float
        """
        return self._y

    @y.setter
    def y(self, y: float):
        """Sets the y of this PlaceParams.

        the y-coordinate of the center of the obstacle placement position  # noqa: E501

        :param y: The y of this PlaceParams.
        :type y: float
        """
        if y is None:
            raise ValueError("Invalid value for `y`, must not be `None`")  # noqa: E501

        self._y = y