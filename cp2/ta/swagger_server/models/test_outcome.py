# coding: utf-8

from __future__ import absolute_import
from swagger_server.models.test_qo_s import TestQoS
from .base_model_ import Model
from datetime import date, datetime
from typing import List, Dict
from ..util import deserialize_model


class TestOutcome(Model):
    """
    NOTE: This class is auto generated by the swagger code generator program.
    Do not edit the class manually.
    """
    def __init__(self, test_id: str=None, time_taken: float=None, timed_out: bool=None, crashed: bool=None, qos: TestQoS=None):
        """
        TestOutcome - a model defined in Swagger

        :param test_id: The test_id of this TestOutcome.
        :type test_id: str
        :param time_taken: The time_taken of this TestOutcome.
        :type time_taken: float
        :param timed_out: The timed_out of this TestOutcome.
        :type timed_out: bool
        :param crashed: The crashed of this TestOutcome.
        :type crashed: bool
        :param qos: The qos of this TestOutcome.
        :type qos: TestQoS
        """
        self.swagger_types = {
            'test_id': str,
            'time_taken': float,
            'timed_out': bool,
            'crashed': bool,
            'qos': TestQoS
        }

        self.attribute_map = {
            'test_id': 'test-id',
            'time_taken': 'time-taken',
            'timed_out': 'timed-out',
            'crashed': 'crashed',
            'qos': 'qos'
        }

        self._test_id = test_id
        self._time_taken = time_taken
        self._timed_out = timed_out
        self._crashed = crashed
        self._qos = qos

    @classmethod
    def from_dict(cls, dikt) -> 'TestOutcome':
        """
        Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The TestOutcome of this TestOutcome.
        :rtype: TestOutcome
        """
        return deserialize_model(dikt, cls)

    @property
    def test_id(self) -> str:
        """
        Gets the test_id of this TestOutcome.
        A unique identifier for the test to which this outcome belongs.

        :return: The test_id of this TestOutcome.
        :rtype: str
        """
        return self._test_id

    @test_id.setter
    def test_id(self, test_id: str):
        """
        Sets the test_id of this TestOutcome.
        A unique identifier for the test to which this outcome belongs.

        :param test_id: The test_id of this TestOutcome.
        :type test_id: str
        """
        if test_id is None:
            raise ValueError("Invalid value for `test_id`, must not be `None`")

        self._test_id = test_id

    @property
    def time_taken(self) -> float:
        """
        Gets the time_taken of this TestOutcome.
        The number of seconds taken to complete the test.

        :return: The time_taken of this TestOutcome.
        :rtype: float
        """
        return self._time_taken

    @time_taken.setter
    def time_taken(self, time_taken: float):
        """
        Sets the time_taken of this TestOutcome.
        The number of seconds taken to complete the test.

        :param time_taken: The time_taken of this TestOutcome.
        :type time_taken: float
        """
        if time_taken is None:
            raise ValueError("Invalid value for `time_taken`, must not be `None`")

        self._time_taken = time_taken

    @property
    def timed_out(self) -> bool:
        """
        Gets the timed_out of this TestOutcome.
        A flag indicating whether or not the test timed out during execution.

        :return: The timed_out of this TestOutcome.
        :rtype: bool
        """
        return self._timed_out

    @timed_out.setter
    def timed_out(self, timed_out: bool):
        """
        Sets the timed_out of this TestOutcome.
        A flag indicating whether or not the test timed out during execution.

        :param timed_out: The timed_out of this TestOutcome.
        :type timed_out: bool
        """
        if timed_out is None:
            raise ValueError("Invalid value for `timed_out`, must not be `None`")

        self._timed_out = timed_out

    @property
    def crashed(self) -> bool:
        """
        Gets the crashed of this TestOutcome.
        A flag indicating whether or not the system crashed during execution of the test.

        :return: The crashed of this TestOutcome.
        :rtype: bool
        """
        return self._crashed

    @crashed.setter
    def crashed(self, crashed: bool):
        """
        Sets the crashed of this TestOutcome.
        A flag indicating whether or not the system crashed during execution of the test.

        :param crashed: The crashed of this TestOutcome.
        :type crashed: bool
        """

        self._crashed = crashed

    @property
    def qos(self) -> TestQoS:
        """
        Gets the qos of this TestOutcome.
        A summary of the quality of service that was observed during the execution of the test.

        :return: The qos of this TestOutcome.
        :rtype: TestQoS
        """
        return self._qos

    @qos.setter
    def qos(self, qos: TestQoS):
        """
        Sets the qos of this TestOutcome.
        A summary of the quality of service that was observed during the execution of the test.

        :param qos: The qos of this TestOutcome.
        :type qos: TestQoS
        """

        self._qos = qos

