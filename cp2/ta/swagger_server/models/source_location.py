# coding: utf-8

from __future__ import absolute_import
from .base_model_ import Model
from datetime import date, datetime
from typing import List, Dict
from ..util import deserialize_model


class SourceLocation(Model):
    """
    NOTE: This class is auto generated by the swagger code generator program.
    Do not edit the class manually.
    """
    def __init__(self, file: str=None, offset: int=None):
        """
        SourceLocation - a model defined in Swagger

        :param file: The file of this SourceLocation.
        :type file: str
        :param offset: The offset of this SourceLocation.
        :type offset: int
        """
        self.swagger_types = {
            'file': str,
            'offset': int
        }

        self.attribute_map = {
            'file': 'file',
            'offset': 'offset'
        }

        self._file = file
        self._offset = offset

    @classmethod
    def from_dict(cls, dikt) -> 'SourceLocation':
        """
        Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The SourceLocation of this SourceLocation.
        :rtype: SourceLocation
        """
        return deserialize_model(dikt, cls)

    @property
    def file(self) -> str:
        """
        Gets the file of this SourceLocation.
        The file at which this source location resides.

        :return: The file of this SourceLocation.
        :rtype: str
        """
        return self._file

    @file.setter
    def file(self, file: str):
        """
        Sets the file of this SourceLocation.
        The file at which this source location resides.

        :param file: The file of this SourceLocation.
        :type file: str
        """
        if file is None:
            raise ValueError("Invalid value for `file`, must not be `None`")

        self._file = file

    @property
    def offset(self) -> int:
        """
        Gets the offset of this SourceLocation.
        The character offset between the start of the file and this location.

        :return: The offset of this SourceLocation.
        :rtype: int
        """
        return self._offset

    @offset.setter
    def offset(self, offset: int):
        """
        Sets the offset of this SourceLocation.
        The character offset between the start of the file and this location.

        :param offset: The offset of this SourceLocation.
        :type offset: int
        """
        if offset is None:
            raise ValueError("Invalid value for `offset`, must not be `None`")
        if offset is not None and offset < 0:
            raise ValueError("Invalid value for `offset`, must be a value greater than or equal to `0`")

        self._offset = offset
