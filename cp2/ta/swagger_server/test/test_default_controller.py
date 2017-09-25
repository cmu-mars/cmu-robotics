# coding: utf-8

from __future__ import absolute_import

from swagger_server.models.inline_response200 import InlineResponse200
from swagger_server.models.inline_response400 import InlineResponse400
from swagger_server.models.parameters import Parameters
from swagger_server.models.parameters1 import Parameters1
from swagger_server.models.parameters2 import Parameters2
from . import BaseTestCase
from six import BytesIO
from flask import json


class TestDefaultController(BaseTestCase):
    """ DefaultController integration test stubs """

    def test_adapt_post(self):
        """
        Test case for adapt_post

        
        """
        Parameters = Parameters1()
        response = self.client.open('/adapt',
                                    method='POST',
                                    data=json.dumps(Parameters),
                                    content_type='application/json')
        self.assert200(response, "Response body is : " + response.data.decode('utf-8'))

    def test_observe_get(self):
        """
        Test case for observe_get

        
        """
        response = self.client.open('/observe',
                                    method='GET')
        self.assert200(response, "Response body is : " + response.data.decode('utf-8'))

    def test_perturb_post(self):
        """
        Test case for perturb_post

        
        """
        Parameters = Parameters2()
        response = self.client.open('/perturb',
                                    method='POST',
                                    data=json.dumps(Parameters),
                                    content_type='application/json')
        self.assert200(response, "Response body is : " + response.data.decode('utf-8'))

    def test_perturbations_get(self):
        """
        Test case for perturbations_get

        
        """
        Parameters = Parameters()
        response = self.client.open('/perturbations',
                                    method='GET',
                                    data=json.dumps(Parameters),
                                    content_type='application/json')
        self.assert200(response, "Response body is : " + response.data.decode('utf-8'))


if __name__ == '__main__':
    import unittest
    unittest.main()
