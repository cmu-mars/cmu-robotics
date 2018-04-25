# coding: utf-8

from __future__ import absolute_import
from datetime import date, datetime  # noqa: F401

from typing import List, Dict  # noqa: F401

from swagger_server.models.base_model_ import Model
from swagger_server import util


class SourceLine(Model):
    """NOTE: This class is auto generated by the swagger code generator program.

    Do not edit the class manually.
    """

    def __init__(self, file: str=None, number: int=None):  # noqa: E501
        """SourceLine - a model defined in Swagger

        :param file: The file of this SourceLine.  # noqa: E501
        :type file: str
        :param number: The number of this SourceLine.  # noqa: E501
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
        """Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The SourceLine of this SourceLine.  # noqa: E501
        :rtype: SourceLine
        """
        return util.deserialize_model(dikt, cls)

    @property
    def file(self) -> str:
        """Gets the file of this SourceLine.

        The file to which this line belongs.  # noqa: E501

        :return: The file of this SourceLine.
        :rtype: str
        """
        return self._file

    @file.setter
    def file(self, file: str):
        """Sets the file of this SourceLine.

        The file to which this line belongs.  # noqa: E501

        :param file: The file of this SourceLine.
        :type file: str
        """
        if file is None:
            raise ValueError("Invalid value for `file`, must not be `None`")  # noqa: E501

        self._file = file

    @property
    def number(self) -> int:
        """Gets the number of this SourceLine.

        The one-indexed number of this line in the file.  # noqa: E501

        :return: The number of this SourceLine.
        :rtype: int
        """
        return self._number

    @number.setter
    def number(self, number: int):
        """Sets the number of this SourceLine.

        The one-indexed number of this line in the file.  # noqa: E501

        :param number: The number of this SourceLine.
        :type number: int
        """
        if number is None:
            raise ValueError("Invalid value for `number`, must not be `None`")  # noqa: E501
        if number is not None and number < 1:  # noqa: E501
            raise ValueError("Invalid value for `number`, must be a value greater than or equal to `1`")  # noqa: E501

        self._number = number
