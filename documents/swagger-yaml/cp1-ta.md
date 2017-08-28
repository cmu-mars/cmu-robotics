# cmu mars brass ta: phase 2, cp1


<a name="overview"></a>
## Overview

### Version information
*Version* : 0.1


### URI scheme
*Host* : brass-ta  
*Schemes* : HTTP




<a name="paths"></a>
## Paths

<a name="perturb-battery-post"></a>
### POST /perturb/battery

#### Description
set the level of the battery in a currently running test


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#perturb-battery-post-parameters)|

<a name="perturb-battery-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*required*|the level to which the battery should be set, in mWh. TODO -- are those the right constants? i have no idea; does it matter if battery sets are monotonically decreasing  <br>**Minimum value** : `16000`  <br>**Maximum value** : `32000`|number|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the battery has been set to the requested level|[Response 200](#perturb-battery-post-response-200)|
|**400**|an error was encountered while setting the battery|[Response 400](#perturb-battery-post-response-400)|

<a name="perturb-battery-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**sim_time**  <br>*required*|the simulation time when the battery was set|integer|

<a name="perturb-battery-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*required*|human readable info about what went wrong|string|


<a name="perturb-place_obstacle-post"></a>
### POST /perturb/place_obstacle

#### Description
if the test is running, then place an obstacle on the map


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#perturb-place_obstacle-post-parameters)|

<a name="perturb-place_obstacle-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**kind**  <br>*required*|the name of the shape of object to be placed. TODO -- make this an enum not a string|string|
|**x**  <br>*required*|the x-coordinate of the center of the obstacle placement position|number (float)|
|**y**  <br>*required*|the y-coordinate of the center of the obstacle placement position|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the obstacle has been placed in the running test|[Response 200](#perturb-place_obstacle-post-response-200)|
|**400**|an error was encountered while placing the obstacle.|[Response 400](#perturb-place_obstacle-post-response-400)|

<a name="perturb-place_obstacle-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**botright_x**  <br>*required*|the x-coordinate of the bottom right corner of the bounding box of the placed obstacle|number (float)|
|**botright_y**  <br>*required*|the y-coordinate of the bottom right corner of the bounding box of the placed obstacle|number (float)|
|**obstacleid**  <br>*required*|a unique identifier for this particular placed obstacle, so that it can be removed in the future|string|
|**sim_time**  <br>*required*|the simulation time when the obstacle was placed|integer|
|**topleft_x**  <br>*required*|the x-coordinate of the top left corner of the bounding box of the placed obstacle|number (float)|
|**topleft_y**  <br>*required*|the y-coordinate of the top left corner of the bounding box of the placed obstacle|number (float)|

<a name="perturb-place_obstacle-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**cause**  <br>*required*|a reason for the error condition|enum (bad_coordiantes, other_error)|
|**message**  <br>*required*|human readable info about what went wrong|string|


<a name="perturb-remove_obstacle-post"></a>
### POST /perturb/remove_obstacle

#### Description
if the test is running, remove a previously placed obstacle from the map


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#perturb-remove_obstacle-post-parameters)|

<a name="perturb-remove_obstacle-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**obstacleid**  <br>*required*|the obstacle ID given by /perturb/place_obstacle of the obstacle to be removed|string|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the obstacle has been removed from the running test|[Response 200](#perturb-remove_obstacle-post-response-200)|
|**400**|an error was encountered while removing the obstacle.|[Response 400](#perturb-remove_obstacle-post-response-400)|

<a name="perturb-remove_obstacle-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**sim_time**  <br>*required*|the simulation time when the obstacle was placed|integer|

<a name="perturb-remove_obstacle-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**cause**  <br>*required*|a reason for the error condition|enum (bad_obstacleid, other_error)|
|**message**  <br>*required*|human readable info about what went wrong|string|







