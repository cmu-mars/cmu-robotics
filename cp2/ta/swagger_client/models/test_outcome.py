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


class TestOutcome(object):
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
        'test_id': 'str',
        'time_taken': 'float',
        'timed_out': 'bool',
        'crashed': 'bool',
        'passed': 'bool',
        'qos': 'TestQoS'
    }

    attribute_map = {
        'test_id': 'test-id',
        'time_taken': 'time-taken',
        'timed_out': 'timed-out',
        'crashed': 'crashed',
        'passed': 'passed',
        'qos': 'qos'
    }

    def __init__(self, test_id=None, time_taken=None, timed_out=None, crashed=None, passed=None, qos=None):
        """
        TestOutcome - a model defined in Swagger
        """

        self._test_id = None
        self._time_taken = None
        self._timed_out = None
        self._crashed = None
        self._passed = None
        self._qos = None

        self.test_id = test_id
        self.time_taken = time_taken
        self.timed_out = timed_out
        if crashed is not None:
          self.crashed = crashed
        self.passed = passed
        if qos is not None:
          self.qos = qos

    @property
    def test_id(self):
        """
        Gets the test_id of this TestOutcome.
        A unique identifier for the test to which this outcome belongs.

        :return: The test_id of this TestOutcome.
        :rtype: str
        """
        return self._test_id

    @test_id.setter
    def test_id(self, test_id):
        """
        Sets the test_id of this TestOutcome.
        A unique identifier for the test to which this outcome belongs.

        :param test_id: The test_id of this TestOutcome.
        :type: str
        """
        if test_id is None:
            raise ValueError("Invalid value for `test_id`, must not be `None`")

        self._test_id = test_id

    @property
    def time_taken(self):
        """
        Gets the time_taken of this TestOutcome.
        The number of seconds taken to complete the test.

        :return: The time_taken of this TestOutcome.
        :rtype: float
        """
        return self._time_taken

    @time_taken.setter
    def time_taken(self, time_taken):
        """
        Sets the time_taken of this TestOutcome.
        The number of seconds taken to complete the test.

        :param time_taken: The time_taken of this TestOutcome.
        :type: float
        """
        if time_taken is None:
            raise ValueError("Invalid value for `time_taken`, must not be `None`")
        if time_taken is not None and time_taken < 0:
            raise ValueError("Invalid value for `time_taken`, must be a value greater than or equal to `0`")

        self._time_taken = time_taken

    @property
    def timed_out(self):
        """
        Gets the timed_out of this TestOutcome.
        A flag indicating whether or not the test timed out during execution.

        :return: The timed_out of this TestOutcome.
        :rtype: bool
        """
        return self._timed_out

    @timed_out.setter
    def timed_out(self, timed_out):
        """
        Sets the timed_out of this TestOutcome.
        A flag indicating whether or not the test timed out during execution.

        :param timed_out: The timed_out of this TestOutcome.
        :type: bool
        """
        if timed_out is None:
            raise ValueError("Invalid value for `timed_out`, must not be `None`")

        self._timed_out = timed_out

    @property
    def crashed(self):
        """
        Gets the crashed of this TestOutcome.
        A flag indicating whether or not the system crashed during execution of the test.

        :return: The crashed of this TestOutcome.
        :rtype: bool
        """
        return self._crashed

    @crashed.setter
    def crashed(self, crashed):
        """
        Sets the crashed of this TestOutcome.
        A flag indicating whether or not the system crashed during execution of the test.

        :param crashed: The crashed of this TestOutcome.
        :type: bool
        """

        self._crashed = crashed

    @property
    def passed(self):
        """
        Gets the passed of this TestOutcome.
        A flag indicating whether or not the system passed the execution of the test

        :return: The passed of this TestOutcome.
        :rtype: bool
        """
        return self._passed

    @passed.setter
    def passed(self, passed):
        """
        Sets the passed of this TestOutcome.
        A flag indicating whether or not the system passed the execution of the test

        :param passed: The passed of this TestOutcome.
        :type: bool
        """
        if passed is None:
            raise ValueError("Invalid value for `passed`, must not be `None`")

        self._passed = passed

    @property
    def qos(self):
        """
        Gets the qos of this TestOutcome.
        A summary of the quality of service that was observed during the execution of the test.

        :return: The qos of this TestOutcome.
        :rtype: TestQoS
        """
        return self._qos

    @qos.setter
    def qos(self, qos):
        """
        Sets the qos of this TestOutcome.
        A summary of the quality of service that was observed during the execution of the test.

        :param qos: The qos of this TestOutcome.
        :type: TestQoS
        """

        self._qos = qos

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
        if not isinstance(other, TestOutcome):
            return False

        return self.__dict__ == other.__dict__

    def __ne__(self, other):
        """
        Returns true if both objects are not equal
        """
        return not self == other