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


class InlineResponse200(object):
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
        'bugzoo_server_urls': 'list[str]'
    }

    attribute_map = {
        'bugzoo_server_urls': 'bugzoo-server-urls'
    }

    def __init__(self, bugzoo_server_urls=None):  # noqa: E501
        """InlineResponse200 - a model defined in Swagger"""  # noqa: E501

        self._bugzoo_server_urls = None
        self.discriminator = None

        if bugzoo_server_urls is not None:
            self.bugzoo_server_urls = bugzoo_server_urls

    @property
    def bugzoo_server_urls(self):
        """Gets the bugzoo_server_urls of this InlineResponse200.  # noqa: E501

        A list of the base URLs for all BugZoo servers that have been made available for the purpose of evaluating candidate patches.  # noqa: E501

        :return: The bugzoo_server_urls of this InlineResponse200.  # noqa: E501
        :rtype: list[str]
        """
        return self._bugzoo_server_urls

    @bugzoo_server_urls.setter
    def bugzoo_server_urls(self, bugzoo_server_urls):
        """Sets the bugzoo_server_urls of this InlineResponse200.

        A list of the base URLs for all BugZoo servers that have been made available for the purpose of evaluating candidate patches.  # noqa: E501

        :param bugzoo_server_urls: The bugzoo_server_urls of this InlineResponse200.  # noqa: E501
        :type: list[str]
        """

        self._bugzoo_server_urls = bugzoo_server_urls

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
        if not isinstance(other, InlineResponse200):
            return False

        return self.__dict__ == other.__dict__

    def __ne__(self, other):
        """Returns true if both objects are not equal"""
        return not self == other
