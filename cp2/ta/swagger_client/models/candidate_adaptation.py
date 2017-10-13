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


class CandidateAdaptation(object):
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
        'diff': 'str',
        'compilation_outcome': 'CompilationOutcome',
        'degradation': 'Degradation',
        'test_outcomes': 'list[TestOutcome]'
    }

    attribute_map = {
        'diff': 'diff',
        'compilation_outcome': 'compilation-outcome',
        'degradation': 'degradation',
        'test_outcomes': 'test-outcomes'
    }

    def __init__(self, diff=None, compilation_outcome=None, degradation=None, test_outcomes=None):
        """
        CandidateAdaptation - a model defined in Swagger
        """

        self._diff = None
        self._compilation_outcome = None
        self._degradation = None
        self._test_outcomes = None

        self.diff = diff
        self.compilation_outcome = compilation_outcome
        self.degradation = degradation
        self.test_outcomes = test_outcomes

    @property
    def diff(self):
        """
        Gets the diff of this CandidateAdaptation.
        A description of the change to the code, given in the form of a diff.

        :return: The diff of this CandidateAdaptation.
        :rtype: str
        """
        return self._diff

    @diff.setter
    def diff(self, diff):
        """
        Sets the diff of this CandidateAdaptation.
        A description of the change to the code, given in the form of a diff.

        :param diff: The diff of this CandidateAdaptation.
        :type: str
        """
        if diff is None:
            raise ValueError("Invalid value for `diff`, must not be `None`")

        self._diff = diff

    @property
    def compilation_outcome(self):
        """
        Gets the compilation_outcome of this CandidateAdaptation.
        A description of the outcome of attempting to compile this adaptation.

        :return: The compilation_outcome of this CandidateAdaptation.
        :rtype: CompilationOutcome
        """
        return self._compilation_outcome

    @compilation_outcome.setter
    def compilation_outcome(self, compilation_outcome):
        """
        Sets the compilation_outcome of this CandidateAdaptation.
        A description of the outcome of attempting to compile this adaptation.

        :param compilation_outcome: The compilation_outcome of this CandidateAdaptation.
        :type: CompilationOutcome
        """
        if compilation_outcome is None:
            raise ValueError("Invalid value for `compilation_outcome`, must not be `None`")

        self._compilation_outcome = compilation_outcome

    @property
    def degradation(self):
        """
        Gets the degradation of this CandidateAdaptation.
        A description of the level of degradation that was observed when this adaptation was applied.

        :return: The degradation of this CandidateAdaptation.
        :rtype: Degradation
        """
        return self._degradation

    @degradation.setter
    def degradation(self, degradation):
        """
        Sets the degradation of this CandidateAdaptation.
        A description of the level of degradation that was observed when this adaptation was applied.

        :param degradation: The degradation of this CandidateAdaptation.
        :type: Degradation
        """
        if degradation is None:
            raise ValueError("Invalid value for `degradation`, must not be `None`")

        self._degradation = degradation

    @property
    def test_outcomes(self):
        """
        Gets the test_outcomes of this CandidateAdaptation.
        A summary of the outcomes for each of the test cases that this adaptation was evaluated against.

        :return: The test_outcomes of this CandidateAdaptation.
        :rtype: list[TestOutcome]
        """
        return self._test_outcomes

    @test_outcomes.setter
    def test_outcomes(self, test_outcomes):
        """
        Sets the test_outcomes of this CandidateAdaptation.
        A summary of the outcomes for each of the test cases that this adaptation was evaluated against.

        :param test_outcomes: The test_outcomes of this CandidateAdaptation.
        :type: list[TestOutcome]
        """
        if test_outcomes is None:
            raise ValueError("Invalid value for `test_outcomes`, must not be `None`")

        self._test_outcomes = test_outcomes

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
        if not isinstance(other, CandidateAdaptation):
            return False

        return self.__dict__ == other.__dict__

    def __ne__(self, other):
        """
        Returns true if both objects are not equal
        """
        return not self == other