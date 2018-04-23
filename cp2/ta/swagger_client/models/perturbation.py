# coding: utf-8

"""
    cmu mars brass th: phase 2, cp2

    No description provided (generated by Swagger Codegen https://github.com/swagger-api/swagger-codegen)  # noqa: E501

    OpenAPI spec version: 0.1
    
    Generated by: https://github.com/swagger-api/swagger-codegen.git
"""


import pprint
import re  # noqa: F401

import six

from swagger_client.models.perturbation_kind import PerturbationKind  # noqa: F401,E501
from swagger_client.models.source_range import SourceRange  # noqa: F401,E501


class Perturbation(object):
    """NOTE: This class is auto generated by the swagger code generator program.

    Do not edit the class manually.
    """

    """
    Attributes:
      swagger_types (dict): The key is attribute name
                            and the value is attribute type.
      attribute_map (dict): The key is attribute name
                            and the value is json key in definition.
    """
    swagger_types = {
        'kind': 'PerturbationKind',
        'at': 'SourceRange',
        'replacement': 'str'
    }

    attribute_map = {
        'kind': 'kind',
        'at': 'at',
        'replacement': 'replacement'
    }

    def __init__(self, kind=None, at=None, replacement=None):  # noqa: E501
        """Perturbation - a model defined in Swagger"""  # noqa: E501

        self._kind = None
        self._at = None
        self._replacement = None
        self.discriminator = None

        self.kind = kind
        self.at = at
        if replacement is not None:
            self.replacement = replacement

    @property
    def kind(self):
        """Gets the kind of this Perturbation.  # noqa: E501


        :return: The kind of this Perturbation.  # noqa: E501
        :rtype: PerturbationKind
        """
        return self._kind

    @kind.setter
    def kind(self, kind):
        """Sets the kind of this Perturbation.


        :param kind: The kind of this Perturbation.  # noqa: E501
        :type: PerturbationKind
        """
        if kind is None:
            raise ValueError("Invalid value for `kind`, must not be `None`")  # noqa: E501

        self._kind = kind

    @property
    def at(self):
        """Gets the at of this Perturbation.  # noqa: E501

        The range of code that is deleted or replaced by the perturbation.  # noqa: E501

        :return: The at of this Perturbation.  # noqa: E501
        :rtype: SourceRange
        """
        return self._at

    @at.setter
    def at(self, at):
        """Sets the at of this Perturbation.

        The range of code that is deleted or replaced by the perturbation.  # noqa: E501

        :param at: The at of this Perturbation.  # noqa: E501
        :type: SourceRange
        """
        if at is None:
            raise ValueError("Invalid value for `at`, must not be `None`")  # noqa: E501

        self._at = at

    @property
    def replacement(self):
        """Gets the replacement of this Perturbation.  # noqa: E501

        The body of the source code that should replaced the source code given by the location range associated with this perturbation.  # noqa: E501

        :return: The replacement of this Perturbation.  # noqa: E501
        :rtype: str
        """
        return self._replacement

    @replacement.setter
    def replacement(self, replacement):
        """Sets the replacement of this Perturbation.

        The body of the source code that should replaced the source code given by the location range associated with this perturbation.  # noqa: E501

        :param replacement: The replacement of this Perturbation.  # noqa: E501
        :type: str
        """

        self._replacement = replacement

    def to_dict(self):
        """Returns the model properties as a dict"""
        result = {}

        for attr, _ in six.iteritems(self.swagger_types):
            value = getattr(self, attr)
            if isinstance(value, list):
                result[attr] = list(map(
                    lambda x: x.to_dict() if hasattr(x, "to_dict") else x,
                    value
                ))
            elif hasattr(value, "to_dict"):
                result[attr] = value.to_dict()
            elif isinstance(value, dict):
                result[attr] = dict(map(
                    lambda item: (item[0], item[1].to_dict())
                    if hasattr(item[1], "to_dict") else item,
                    value.items()
                ))
            else:
                result[attr] = value

        return result

    def to_str(self):
        """Returns the string representation of the model"""
        return pprint.pformat(self.to_dict())

    def __repr__(self):
        """For `print` and `pprint`"""
        return self.to_str()

    def __eq__(self, other):
        """Returns true if both objects are equal"""
        if not isinstance(other, Perturbation):
            return False

        return self.__dict__ == other.__dict__

    def __ne__(self, other):
        """Returns true if both objects are not equal"""
        return not self == other
