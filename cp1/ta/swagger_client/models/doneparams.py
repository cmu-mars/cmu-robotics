# coding: utf-8

"""
    cmu mars brass th: phase 2, cp1

    No description provided (generated by Swagger Codegen https://github.com/swagger-api/swagger-codegen)  # noqa: E501

    OpenAPI spec version: 0.1
    
    Generated by: https://github.com/swagger-api/swagger-codegen.git
"""


import pprint
import re  # noqa: F401

import six

from swagger_client.models.done_tasksfinished import DoneTasksfinished  # noqa: F401,E501


class Doneparams(object):
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
        'x': 'float',
        'y': 'float',
        'charge': 'int',
        'sim_time': 'int',
        'tasks_finished': 'list[DoneTasksfinished]',
        'outcome': 'str',
        'message': 'str'
    }

    attribute_map = {
        'x': 'x',
        'y': 'y',
        'charge': 'charge',
        'sim_time': 'sim-time',
        'tasks_finished': 'tasks-finished',
        'outcome': 'outcome',
        'message': 'message'
    }

    def __init__(self, x=None, y=None, charge=None, sim_time=None, tasks_finished=None, outcome=None, message=None):  # noqa: E501
        """Doneparams - a model defined in Swagger"""  # noqa: E501

        self._x = None
        self._y = None
        self._charge = None
        self._sim_time = None
        self._tasks_finished = None
        self._outcome = None
        self._message = None
        self.discriminator = None

        self.x = x
        self.y = y
        self.charge = charge
        self.sim_time = sim_time
        self.tasks_finished = tasks_finished
        self.outcome = outcome
        if message is not None:
            self.message = message

    @property
    def x(self):
        """Gets the x of this Doneparams.  # noqa: E501

        final x-coordinate of the turtlebot  # noqa: E501

        :return: The x of this Doneparams.  # noqa: E501
        :rtype: float
        """
        return self._x

    @x.setter
    def x(self, x):
        """Sets the x of this Doneparams.

        final x-coordinate of the turtlebot  # noqa: E501

        :param x: The x of this Doneparams.  # noqa: E501
        :type: float
        """
        if x is None:
            raise ValueError("Invalid value for `x`, must not be `None`")  # noqa: E501

        self._x = x

    @property
    def y(self):
        """Gets the y of this Doneparams.  # noqa: E501

        final y-coordinate of the turtlebot  # noqa: E501

        :return: The y of this Doneparams.  # noqa: E501
        :rtype: float
        """
        return self._y

    @y.setter
    def y(self, y):
        """Sets the y of this Doneparams.

        final y-coordinate of the turtlebot  # noqa: E501

        :param y: The y of this Doneparams.  # noqa: E501
        :type: float
        """
        if y is None:
            raise ValueError("Invalid value for `y`, must not be `None`")  # noqa: E501

        self._y = y

    @property
    def charge(self):
        """Gets the charge of this Doneparams.  # noqa: E501

        final charge measure of the turtlebot. cannot be more than the maximum specified in the response from `/ready`.  # noqa: E501

        :return: The charge of this Doneparams.  # noqa: E501
        :rtype: int
        """
        return self._charge

    @charge.setter
    def charge(self, charge):
        """Sets the charge of this Doneparams.

        final charge measure of the turtlebot. cannot be more than the maximum specified in the response from `/ready`.  # noqa: E501

        :param charge: The charge of this Doneparams.  # noqa: E501
        :type: int
        """
        if charge is None:
            raise ValueError("Invalid value for `charge`, must not be `None`")  # noqa: E501
        if charge is not None and charge < 0:  # noqa: E501
            raise ValueError("Invalid value for `charge`, must be a value greater than or equal to `0`")  # noqa: E501

        self._charge = charge

    @property
    def sim_time(self):
        """Gets the sim_time of this Doneparams.  # noqa: E501

        the final internal simulation time  # noqa: E501

        :return: The sim_time of this Doneparams.  # noqa: E501
        :rtype: int
        """
        return self._sim_time

    @sim_time.setter
    def sim_time(self, sim_time):
        """Sets the sim_time of this Doneparams.

        the final internal simulation time  # noqa: E501

        :param sim_time: The sim_time of this Doneparams.  # noqa: E501
        :type: int
        """
        if sim_time is None:
            raise ValueError("Invalid value for `sim_time`, must not be `None`")  # noqa: E501
        if sim_time is not None and sim_time < 0:  # noqa: E501
            raise ValueError("Invalid value for `sim_time`, must be a value greater than or equal to `0`")  # noqa: E501

        self._sim_time = sim_time

    @property
    def tasks_finished(self):
        """Gets the tasks_finished of this Doneparams.  # noqa: E501

        the names of the waypoints that the turtlebot visited in the order that it visited them as well as the x,y coordinates of the robot and simulation time that it arrived there  # noqa: E501

        :return: The tasks_finished of this Doneparams.  # noqa: E501
        :rtype: list[DoneTasksfinished]
        """
        return self._tasks_finished

    @tasks_finished.setter
    def tasks_finished(self, tasks_finished):
        """Sets the tasks_finished of this Doneparams.

        the names of the waypoints that the turtlebot visited in the order that it visited them as well as the x,y coordinates of the robot and simulation time that it arrived there  # noqa: E501

        :param tasks_finished: The tasks_finished of this Doneparams.  # noqa: E501
        :type: list[DoneTasksfinished]
        """
        if tasks_finished is None:
            raise ValueError("Invalid value for `tasks_finished`, must not be `None`")  # noqa: E501

        self._tasks_finished = tasks_finished

    @property
    def outcome(self):
        """Gets the outcome of this Doneparams.  # noqa: E501

        indicates the reason why the test is over    * at-goal - the turtlebot has reached the goal and               completed the mission objectives    * out-of-battery - the battery on the turtlebot has run                      out, and cannot be charged, so the                      turtlebot cannot make progress    * other-outcome - the test is over for any other                     non-error reason  # noqa: E501

        :return: The outcome of this Doneparams.  # noqa: E501
        :rtype: str
        """
        return self._outcome

    @outcome.setter
    def outcome(self, outcome):
        """Sets the outcome of this Doneparams.

        indicates the reason why the test is over    * at-goal - the turtlebot has reached the goal and               completed the mission objectives    * out-of-battery - the battery on the turtlebot has run                      out, and cannot be charged, so the                      turtlebot cannot make progress    * other-outcome - the test is over for any other                     non-error reason  # noqa: E501

        :param outcome: The outcome of this Doneparams.  # noqa: E501
        :type: str
        """
        if outcome is None:
            raise ValueError("Invalid value for `outcome`, must not be `None`")  # noqa: E501
        allowed_values = ["at-goal", "out-of-battery", "other-outcome"]  # noqa: E501
        if outcome not in allowed_values:
            raise ValueError(
                "Invalid value for `outcome` ({0}), must be one of {1}"  # noqa: E501
                .format(outcome, allowed_values)
            )

        self._outcome = outcome

    @property
    def message(self):
        """Gets the message of this Doneparams.  # noqa: E501

        human-readable text with more information about the end of the test.  # noqa: E501

        :return: The message of this Doneparams.  # noqa: E501
        :rtype: str
        """
        return self._message

    @message.setter
    def message(self, message):
        """Sets the message of this Doneparams.

        human-readable text with more information about the end of the test.  # noqa: E501

        :param message: The message of this Doneparams.  # noqa: E501
        :type: str
        """

        self._message = message

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
        if not isinstance(other, Doneparams):
            return False

        return self.__dict__ == other.__dict__

    def __ne__(self, other):
        """Returns true if both objects are not equal"""
        return not self == other
