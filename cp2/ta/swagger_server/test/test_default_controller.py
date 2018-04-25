# coding: utf-8

from __future__ import absolute_import

from flask import json
from six import BytesIO

from swagger_server.models.error import Error  # noqa: E501
from swagger_server.models.inline_response200 import InlineResponse200  # noqa: E501
from swagger_server.models.inline_response2001 import InlineResponse2001  # noqa: E501
from swagger_server.models.parameters import Parameters  # noqa: E501
from swagger_server.models.perturbation import Perturbation  # noqa: E501
from swagger_server.models.perturbation_params import PerturbationParams  # noqa: E501
from swagger_server.models.source_line import SourceLine  # noqa: E501
from swagger_server.test import BaseTestCase


class TestDefaultController(BaseTestCase):
    """DefaultController integration test stubs"""

    def test_adapt_post(self):
        """Test case for adapt_post

        
        """
        Parameters = Parameters()
        response = self.client.open(
            '/adapt',
            method='POST',
            data=json.dumps(Parameters),
            content_type='application/json')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_files_get(self):
        """Test case for files_get

        
        """
        response = self.client.open(
            '/files',
            method='GET')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_lines_get(self):
        """Test case for lines_get

        
        """
        response = self.client.open(
            '/lines',
            method='GET')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_observe_get(self):
        """Test case for observe_get

        
        """
        response = self.client.open(
            '/observe',
            method='GET')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_perturb_post(self):
        """Test case for perturb_post

        
        """
        perturb_params = Perturbation()
        response = self.client.open(
            '/perturb',
            method='POST',
            data=json.dumps(perturb_params),
            content_type='application/json')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_perturbations_get(self):
        """Test case for perturbations_get

        
        """
        perturbation_params = PerturbationParams()
        response = self.client.open(
            '/perturbations',
            method='GET',
            data=json.dumps(perturbation_params),
            content_type='application/json')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))


if __name__ == '__main__':
    import unittest
    unittest.main()
