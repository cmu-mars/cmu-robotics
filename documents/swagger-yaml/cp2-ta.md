# cmu mars brass ta: phase 2, cp2


<a name="overview"></a>
## Overview

### Version information
*Version* : 0.1


### URI scheme
*Host* : brass-ta  
*Schemes* : HTTP




<a name="paths"></a>
## Paths

<a name="adapt-post"></a>
### POST /adapt

#### Description
Used to trigger the code adaptation process.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#adapt-post-parameters)|

<a name="adapt-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**attemptlimit**  <br>*optional*|An (optional) limit on the number of adaptations that may be attempted.|integer|
|**hint**  <br>*optional*|We could allow hints to be provided to this method? e.g., the shape(s) or location(s) of the fix(es). TODO|string|
|**timelimit**  <br>*optional*|An (optional) time limit for the adaptation process, given in minutes.|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|successfully triggered code adaptation|No Content|
|**400**|encountered an error while triggering code adaptation|[Response 400](#adapt-post-response-400)|

<a name="adapt-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|


<a name="perturb-post"></a>
### POST /perturb

#### Description
Applies a given set of perturbations, provided as a list of JSON objects, to the SUT. This method should be used to prepare a test scenario for evaluation.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#perturb-post-parameters)|

<a name="perturb-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**perturbations**  <br>*required*|a list of perturbations to apply to the code base|< string > array|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|applied the perturbations to the the system successfully|No Content|
|**400**|encountered an error while applying the perturbations to the system|[Response 400](#perturb-post-response-400)|

<a name="perturb-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|


<a name="perturbations-get"></a>
### GET /perturbations

#### Description
Returns a list of possible perturbations of an (optionally) specified kind and complexity that can be performed at a given (set of) location(s) in the program.  This endpoint should be used to select a suitable (set of) perturbation(s) for a test scenario.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#perturbations-get-parameters)|

<a name="perturbations-get-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**file**  <br>*required*|the file in which the fault should be placed|string|
|**kind**  <br>*required*|the "kind" of the fault TODO -- vague|string|
|**line**  <br>*required*|TODO|integer|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|successfully computed the list of possible perturbations|[Response 200](#perturbations-get-response-200)|
|**400**|encountered an error while computing the list of possible perturbations|[Response 400](#perturbations-get-response-400)|

<a name="perturbations-get-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**perturbations**  <br>*optional*|list of perturbations that satisfy the query parameters provided by the request. Each perturbation is described by its Kind, the Location to which it should be applied, and any additional parameters that are required to complete the perturbation (e.g., a replacement statement). TODO -- figure out the actual types here|< string > array|

<a name="perturbations-get-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|


<a name="status-get"></a>
### GET /status

#### Description
Returns the current status of the SUT


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|successfully computed the status of the SUT|No Content|
|**400**|encounted an error while computing the status of the SUT|No Content|







