# coding: utf-8

"""
    cmu mars brass th: phase 2, cp2

    No description provided (generated by Swagger Codegen https://github.com/swagger-api/swagger-codegen)  # noqa: E501

    OpenAPI spec version: 0.1
    
    Generated by: https://github.com/swagger-api/swagger-codegen.git
"""


from __future__ import absolute_import

import re  # noqa: F401

# python 2 and python 3 compatibility library
import six

from swagger_client.api_client import ApiClient


class DefaultApi(object):
    """NOTE: This class is auto generated by the swagger code generator program.

    Do not edit the class manually.
    Ref: https://github.com/swagger-api/swagger-codegen
    """

    def __init__(self, api_client=None):
        if api_client is None:
            api_client = ApiClient()
        self.api_client = api_client

    def done_post(self, parameters, **kwargs):  # noqa: E501
        """done_post  # noqa: E501

        Used to indicate that evaluation of the test scenario has been completed. A summary of the results of the test scenario are provided as a JSON object. The entire summary is contained within `SutFinishedStatus`, as specified by the Lincoln Labs API.  # noqa: E501
        This method makes a synchronous HTTP request by default. To make an
        asynchronous HTTP request, please pass async=True
        >>> thread = api.done_post(parameters, async=True)
        >>> result = thread.get()

        :param async bool
        :param Parameters1 parameters: (required)
        :return: None
                 If the method is called asynchronously,
                 returns the request thread.
        """
        kwargs['_return_http_data_only'] = True
        if kwargs.get('async'):
            return self.done_post_with_http_info(parameters, **kwargs)  # noqa: E501
        else:
            (data) = self.done_post_with_http_info(parameters, **kwargs)  # noqa: E501
            return data

    def done_post_with_http_info(self, parameters, **kwargs):  # noqa: E501
        """done_post  # noqa: E501

        Used to indicate that evaluation of the test scenario has been completed. A summary of the results of the test scenario are provided as a JSON object. The entire summary is contained within `SutFinishedStatus`, as specified by the Lincoln Labs API.  # noqa: E501
        This method makes a synchronous HTTP request by default. To make an
        asynchronous HTTP request, please pass async=True
        >>> thread = api.done_post_with_http_info(parameters, async=True)
        >>> result = thread.get()

        :param async bool
        :param Parameters1 parameters: (required)
        :return: None
                 If the method is called asynchronously,
                 returns the request thread.
        """

        all_params = ['parameters']  # noqa: E501
        all_params.append('async')
        all_params.append('_return_http_data_only')
        all_params.append('_preload_content')
        all_params.append('_request_timeout')

        params = locals()
        for key, val in six.iteritems(params['kwargs']):
            if key not in all_params:
                raise TypeError(
                    "Got an unexpected keyword argument '%s'"
                    " to method done_post" % key
                )
            params[key] = val
        del params['kwargs']
        # verify the required parameter 'parameters' is set
        if ('parameters' not in params or
                params['parameters'] is None):
            raise ValueError("Missing the required parameter `parameters` when calling `done_post`")  # noqa: E501

        collection_formats = {}

        path_params = {}

        query_params = []

        header_params = {}

        form_params = []
        local_var_files = {}

        body_params = None
        if 'parameters' in params:
            body_params = params['parameters']
        # Authentication setting
        auth_settings = []  # noqa: E501

        return self.api_client.call_api(
            '/done', 'POST',
            path_params,
            query_params,
            header_params,
            body=body_params,
            post_params=form_params,
            files=local_var_files,
            response_type=None,  # noqa: E501
            auth_settings=auth_settings,
            async=params.get('async'),
            _return_http_data_only=params.get('_return_http_data_only'),
            _preload_content=params.get('_preload_content', True),
            _request_timeout=params.get('_request_timeout'),
            collection_formats=collection_formats)

    def error_post(self, parameters, **kwargs):  # noqa: E501
        """error_post  # noqa: E501

        Used to indicate that an error has occurred during the preparation or evaluation of a test scenario, or during the start-up of the system under test.  # noqa: E501
        This method makes a synchronous HTTP request by default. To make an
        asynchronous HTTP request, please pass async=True
        >>> thread = api.error_post(parameters, async=True)
        >>> result = thread.get()

        :param async bool
        :param Error parameters: (required)
        :return: None
                 If the method is called asynchronously,
                 returns the request thread.
        """
        kwargs['_return_http_data_only'] = True
        if kwargs.get('async'):
            return self.error_post_with_http_info(parameters, **kwargs)  # noqa: E501
        else:
            (data) = self.error_post_with_http_info(parameters, **kwargs)  # noqa: E501
            return data

    def error_post_with_http_info(self, parameters, **kwargs):  # noqa: E501
        """error_post  # noqa: E501

        Used to indicate that an error has occurred during the preparation or evaluation of a test scenario, or during the start-up of the system under test.  # noqa: E501
        This method makes a synchronous HTTP request by default. To make an
        asynchronous HTTP request, please pass async=True
        >>> thread = api.error_post_with_http_info(parameters, async=True)
        >>> result = thread.get()

        :param async bool
        :param Error parameters: (required)
        :return: None
                 If the method is called asynchronously,
                 returns the request thread.
        """

        all_params = ['parameters']  # noqa: E501
        all_params.append('async')
        all_params.append('_return_http_data_only')
        all_params.append('_preload_content')
        all_params.append('_request_timeout')

        params = locals()
        for key, val in six.iteritems(params['kwargs']):
            if key not in all_params:
                raise TypeError(
                    "Got an unexpected keyword argument '%s'"
                    " to method error_post" % key
                )
            params[key] = val
        del params['kwargs']
        # verify the required parameter 'parameters' is set
        if ('parameters' not in params or
                params['parameters'] is None):
            raise ValueError("Missing the required parameter `parameters` when calling `error_post`")  # noqa: E501

        collection_formats = {}

        path_params = {}

        query_params = []

        header_params = {}

        form_params = []
        local_var_files = {}

        body_params = None
        if 'parameters' in params:
            body_params = params['parameters']
        # HTTP header `Content-Type`
        header_params['Content-Type'] = self.api_client.select_header_content_type(  # noqa: E501
            ['application/json'])  # noqa: E501

        # Authentication setting
        auth_settings = []  # noqa: E501

        return self.api_client.call_api(
            '/error', 'POST',
            path_params,
            query_params,
            header_params,
            body=body_params,
            post_params=form_params,
            files=local_var_files,
            response_type=None,  # noqa: E501
            auth_settings=auth_settings,
            async=params.get('async'),
            _return_http_data_only=params.get('_return_http_data_only'),
            _preload_content=params.get('_preload_content', True),
            _request_timeout=params.get('_request_timeout'),
            collection_formats=collection_formats)

    def ready_post(self, **kwargs):  # noqa: E501
        """ready_post  # noqa: E501

        Used to indicate that the SUT is ready and that testing may begin.  # noqa: E501
        This method makes a synchronous HTTP request by default. To make an
        asynchronous HTTP request, please pass async=True
        >>> thread = api.ready_post(async=True)
        >>> result = thread.get()

        :param async bool
        :return: InlineResponse200
                 If the method is called asynchronously,
                 returns the request thread.
        """
        kwargs['_return_http_data_only'] = True
        if kwargs.get('async'):
            return self.ready_post_with_http_info(**kwargs)  # noqa: E501
        else:
            (data) = self.ready_post_with_http_info(**kwargs)  # noqa: E501
            return data

    def ready_post_with_http_info(self, **kwargs):  # noqa: E501
        """ready_post  # noqa: E501

        Used to indicate that the SUT is ready and that testing may begin.  # noqa: E501
        This method makes a synchronous HTTP request by default. To make an
        asynchronous HTTP request, please pass async=True
        >>> thread = api.ready_post_with_http_info(async=True)
        >>> result = thread.get()

        :param async bool
        :return: InlineResponse200
                 If the method is called asynchronously,
                 returns the request thread.
        """

        all_params = []  # noqa: E501
        all_params.append('async')
        all_params.append('_return_http_data_only')
        all_params.append('_preload_content')
        all_params.append('_request_timeout')

        params = locals()
        for key, val in six.iteritems(params['kwargs']):
            if key not in all_params:
                raise TypeError(
                    "Got an unexpected keyword argument '%s'"
                    " to method ready_post" % key
                )
            params[key] = val
        del params['kwargs']

        collection_formats = {}

        path_params = {}

        query_params = []

        header_params = {}

        form_params = []
        local_var_files = {}

        body_params = None
        # HTTP header `Accept`
        header_params['Accept'] = self.api_client.select_header_accept(
            ['application/json'])  # noqa: E501

        # Authentication setting
        auth_settings = []  # noqa: E501

        return self.api_client.call_api(
            '/ready', 'POST',
            path_params,
            query_params,
            header_params,
            body=body_params,
            post_params=form_params,
            files=local_var_files,
            response_type='InlineResponse200',  # noqa: E501
            auth_settings=auth_settings,
            async=params.get('async'),
            _return_http_data_only=params.get('_return_http_data_only'),
            _preload_content=params.get('_preload_content', True),
            _request_timeout=params.get('_request_timeout'),
            collection_formats=collection_formats)

    def status_post(self, parameters, **kwargs):  # noqa: E501
        """status_post  # noqa: E501

        Used to inform the test harness that a new adaptation has been added to the Pareto set (i.e., a new \"best\" adaptation has been found).  # noqa: E501
        This method makes a synchronous HTTP request by default. To make an
        asynchronous HTTP request, please pass async=True
        >>> thread = api.status_post(parameters, async=True)
        >>> result = thread.get()

        :param async bool
        :param Parameters parameters: (required)
        :return: None
                 If the method is called asynchronously,
                 returns the request thread.
        """
        kwargs['_return_http_data_only'] = True
        if kwargs.get('async'):
            return self.status_post_with_http_info(parameters, **kwargs)  # noqa: E501
        else:
            (data) = self.status_post_with_http_info(parameters, **kwargs)  # noqa: E501
            return data

    def status_post_with_http_info(self, parameters, **kwargs):  # noqa: E501
        """status_post  # noqa: E501

        Used to inform the test harness that a new adaptation has been added to the Pareto set (i.e., a new \"best\" adaptation has been found).  # noqa: E501
        This method makes a synchronous HTTP request by default. To make an
        asynchronous HTTP request, please pass async=True
        >>> thread = api.status_post_with_http_info(parameters, async=True)
        >>> result = thread.get()

        :param async bool
        :param Parameters parameters: (required)
        :return: None
                 If the method is called asynchronously,
                 returns the request thread.
        """

        all_params = ['parameters']  # noqa: E501
        all_params.append('async')
        all_params.append('_return_http_data_only')
        all_params.append('_preload_content')
        all_params.append('_request_timeout')

        params = locals()
        for key, val in six.iteritems(params['kwargs']):
            if key not in all_params:
                raise TypeError(
                    "Got an unexpected keyword argument '%s'"
                    " to method status_post" % key
                )
            params[key] = val
        del params['kwargs']
        # verify the required parameter 'parameters' is set
        if ('parameters' not in params or
                params['parameters'] is None):
            raise ValueError("Missing the required parameter `parameters` when calling `status_post`")  # noqa: E501

        collection_formats = {}

        path_params = {}

        query_params = []

        header_params = {}

        form_params = []
        local_var_files = {}

        body_params = None
        if 'parameters' in params:
            body_params = params['parameters']
        # HTTP header `Content-Type`
        header_params['Content-Type'] = self.api_client.select_header_content_type(  # noqa: E501
            ['application/json'])  # noqa: E501

        # Authentication setting
        auth_settings = []  # noqa: E501

        return self.api_client.call_api(
            '/status', 'POST',
            path_params,
            query_params,
            header_params,
            body=body_params,
            post_params=form_params,
            files=local_var_files,
            response_type=None,  # noqa: E501
            auth_settings=auth_settings,
            async=params.get('async'),
            _return_http_data_only=params.get('_return_http_data_only'),
            _preload_content=params.get('_preload_content', True),
            _request_timeout=params.get('_request_timeout'),
            collection_formats=collection_formats)