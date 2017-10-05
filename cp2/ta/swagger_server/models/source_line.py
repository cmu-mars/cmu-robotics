# coding: utf-8

from __future__ import absolute_import
from .base_model_ import Model
from datetime import date, datetime
from typing import List, Dict
from ..util import deserialize_model


class SourceLine(Model):
    """
    NOTE: This class is auto generated by the swagger code generator program.
    Do not edit the class manually.
    """
    def __init__(self, file: str=None, number: int=None):
        """
        SourceLine - a model defined in Swagger

        :param file: The file of this SourceLine.
        :type file: str
        :param number: The number of this SourceLine.
        :type number: int
        """
        self.swagger_types = {
            'file': str,
            'number': int
        }

        self.attribute_map = {
            'file': 'file',
            'number': 'number'
        }

        self._file = file
        self._number = number

    @classmethod
    def from_dict(cls, dikt) -> 'SourceLine':
        """
        Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The SourceLine of this SourceLine.
        :rtype: SourceLine
        """
        return deserialize_model(dikt, cls)

    @property
    def file(self) -> str:
        """
        Gets the file of this SourceLine.
        The file to which this line belongs.

        :return: The file of this SourceLine.
        :rtype: str
        """
        return self._file

    @file.setter
    def file(self, file: str):
        """
        Sets the file of this SourceLine.
        The file to which this line belongs.

        :param file: The file of this SourceLine.
        :type file: str
        """
        if file is None:
            raise ValueError("Invalid value for `file`, must not be `None`")

        self._file = file

    @property
    def number(self) -> int:
        """
        Gets the number of this SourceLine.
        The one-indexed number of this line in the file.

        :return: The number of this SourceLine.
        :rtype: int
        """
        return self._number

    @number.setter
    def number(self, number: int):
        """
        Sets the number of this SourceLine.
        The one-indexed number of this line in the file.

        :param number: The number of this SourceLine.
        :type number: int
        """
        if number is None:
            raise ValueError("Invalid value for `number`, must not be `None`")
        if number is not None and number < 1:
            raise ValueError("Invalid value for `number`, must be a value greater than or equal to `1`")

        self._number = number
