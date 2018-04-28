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


class CompilationOutcome(object):
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
        'time_taken': 'float',
        'successful': 'bool'
    }

    attribute_map = {
        'time_taken': 'time-taken',
        'successful': 'successful'
    }

    def __init__(self, time_taken=None, successful=None):  # noqa: E501
        """CompilationOutcome - a model defined in Swagger"""  # noqa: E501

        self._time_taken = None
        self._successful = None
        self.discriminator = None

        self.time_taken = time_taken
        self.successful = successful

    @property
    def time_taken(self):
        """Gets the time_taken of this CompilationOutcome.  # noqa: E501

        The number of seconds taken to compile this adaptation.  # noqa: E501

        :return: The time_taken of this CompilationOutcome.  # noqa: E501
        :rtype: float
        """
        return self._time_taken

    @time_taken.setter
    def time_taken(self, time_taken):
        """Sets the time_taken of this CompilationOutcome.

        The number of seconds taken to compile this adaptation.  # noqa: E501

        :param time_taken: The time_taken of this CompilationOutcome.  # noqa: E501
        :type: float
        """
        if time_taken is None:
            raise ValueError("Invalid value for `time_taken`, must not be `None`")  # noqa: E501
        if time_taken is not None and time_taken < 0:  # noqa: E501
            raise ValueError("Invalid value for `time_taken`, must be a value greater than or equal to `0`")  # noqa: E501

        self._time_taken = time_taken

    @property
    def successful(self):
        """Gets the successful of this CompilationOutcome.  # noqa: E501

        A flag indicating whether the compilation of this adaptation was successful or not.  # noqa: E501

        :return: The successful of this CompilationOutcome.  # noqa: E501
        :rtype: bool
        """
        return self._successful

    @successful.setter
    def successful(self, successful):
        """Sets the successful of this CompilationOutcome.

        A flag indicating whether the compilation of this adaptation was successful or not.  # noqa: E501

        :param successful: The successful of this CompilationOutcome.  # noqa: E501
        :type: bool
        """
        if successful is None:
            raise ValueError("Invalid value for `successful`, must not be `None`")  # noqa: E501

        self._successful = successful

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
        if not isinstance(other, CompilationOutcome):
            return False

        return self.__dict__ == other.__dict__

    def __ne__(self, other):
        """Returns true if both objects are not equal"""
        return not self == other
