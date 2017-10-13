# coding: utf-8

"""
    cmu mars brass th: phase 2, cp2

    No description provided (generated by Swagger Codegen https://github.com/swagger-api/swagger-codegen)

    OpenAPI spec version: 0.1
    
    Generated by: https://github.com/swagger-api/swagger-codegen.git
"""


from pprint import pformat
from six import iteritems
import re


class InlineResponse200(object):
    """
    NOTE: This class is auto generated by the swagger code generator program.
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
        'docker_ip': 'str'
    }

    attribute_map = {
        'docker_ip': 'docker-ip'
    }

    def __init__(self, docker_ip=None):
        """
        InlineResponse200 - a model defined in Swagger
        """

        self._docker_ip = None

        self.docker_ip = docker_ip

    @property
    def docker_ip(self):
        """
        Gets the docker_ip of this InlineResponse200.
        the IP address to connect to in order to allocate new worker containers

        :return: The docker_ip of this InlineResponse200.
        :rtype: str
        """
        return self._docker_ip

    @docker_ip.setter
    def docker_ip(self, docker_ip):
        """
        Sets the docker_ip of this InlineResponse200.
        the IP address to connect to in order to allocate new worker containers

        :param docker_ip: The docker_ip of this InlineResponse200.
        :type: str
        """
        if docker_ip is None:
            raise ValueError("Invalid value for `docker_ip`, must not be `None`")

        self._docker_ip = docker_ip

    def to_dict(self):
        """
        Returns the model properties as a dict
        """
        result = {}

        for attr, _ in iteritems(self.swagger_types):
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
        """
        Returns the string representation of the model
        """
        return pformat(self.to_dict())

    def __repr__(self):
        """
        For `print` and `pprint`
        """
        return self.to_str()

    def __eq__(self, other):
        """
        Returns true if both objects are equal
        """
        if not isinstance(other, InlineResponse200):
            return False

        return self.__dict__ == other.__dict__

    def __ne__(self, other):
        """
        Returns true if both objects are not equal
        """
        return not self == other