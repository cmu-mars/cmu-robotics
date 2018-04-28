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

<a name="observe-get"></a>
### GET /observe

#### Description
observe some of the current state of the robot for visualization and invariant checking for perturbation end points. n.b. this information is to be used strictly in a passive way; it is not to be used for evaluation of the test at all.


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|successfully computed the observation|[Response 200](#observe-get-response-200)|
|**400**|encountered an error while computing the observation|No Content|

<a name="observe-get-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**battery**  <br>*required*|the current charge of the battery, in mWh  <br>**Minimum value** : `0`|integer|
|**sim-time**  <br>*required*|the time when this observation was computed, in simulation seconds  <br>**Minimum value** : `0`|integer|
|**x**  <br>*required*|the current x coordinate of the bot. must be within the boundaries of the map.|number (float)|
|**y**  <br>*required*|the current y coordinate of the bot. must be within the boundaries of the map.|number (float)|


<a name="perturb-battery-post"></a>
### POST /perturb/battery

#### Description
set the level of the battery in a currently running test. consistent with the monotonicity requirement for the power model, this cannot be more than the current amount of charge in the battery.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**BatteryParams**  <br>*optional*|[BatteryParams](#perturb-battery-post-batteryparams)|

<a name="perturb-battery-post-batteryparams"></a>
**BatteryParams**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*required*|the level to which the battery should be set, in mWh. cannot be more than the maximum charge for the power model specified in the THs response to `/ready`.  <br>**Minimum value** : `0`|number|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the battery has been set to the requested level|[Response 200](#perturb-battery-post-response-200)|
|**400**|an error was encountered while setting the battery|[Response 400](#perturb-battery-post-response-400)|

<a name="perturb-battery-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**sim-time**  <br>*required*|the simulation time when the battery was set|integer|

<a name="perturb-battery-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*required*|human readable info about what went wrong|string|


<a name="perturb-place-obstacle-post"></a>
### POST /perturb/place-obstacle

#### Description
if the test is running, then place an instance of the obstacle on the map


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**PlaceParams**  <br>*optional*|[PlaceParams](#perturb-place-obstacle-post-placeparams)|

<a name="perturb-place-obstacle-post-placeparams"></a>
**PlaceParams**

|Name|Description|Schema|
|---|---|---|
|**x**  <br>*required*|the x-coordinate of the center of the obstacle placement position|number (float)|
|**y**  <br>*required*|the y-coordinate of the center of the obstacle placement position|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the obstacle has been placed in the running test|[Response 200](#perturb-place-obstacle-post-response-200)|
|**400**|an error was encountered while placing the obstacle.|[Response 400](#perturb-place-obstacle-post-response-400)|

<a name="perturb-place-obstacle-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**obstacleid**  <br>*required*|a unique identifier for this particular placed obstacle, so that it can be removed in the future|string|
|**sim-time**  <br>*required*|the simulation time when the obstacle was placed|integer|

<a name="perturb-place-obstacle-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**cause**  <br>*required*|a reason for the error condition|enum (bad-coordiantes, other-error)|
|**message**  <br>*required*|human readable info about what went wrong|string|


<a name="perturb-remove-obstacle-post"></a>
### POST /perturb/remove-obstacle

#### Description
if the test is running, remove a previously placed obstacle from the map


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**RemoveParams**  <br>*optional*|[RemoveParams](#perturb-remove-obstacle-post-removeparams)|

<a name="perturb-remove-obstacle-post-removeparams"></a>
**RemoveParams**

|Name|Description|Schema|
|---|---|---|
|**obstacleid**  <br>*required*|the obstacle ID given by /perturb/place-obstacle of the obstacle to be removed.|string|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the obstacle has been removed from the running test|[Response 200](#perturb-remove-obstacle-post-response-200)|
|**400**|an error was encountered while removing the obstacle.|[Response 400](#perturb-remove-obstacle-post-response-400)|

<a name="perturb-remove-obstacle-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**sim-time**  <br>*required*|the simulation time when the obstacle was placed|integer|

<a name="perturb-remove-obstacle-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**cause**  <br>*required*|a reason for the error condition. `bad-obstacleid` is used if this endpoint is given a obstacleid in its parameters that was not given out by place-obstacle; `other-error` is used in all other instances.|enum (bad-obstacleid, other-error)|
|**message**  <br>*required*|human readable info about what went wrong|string|


<a name="start-post"></a>
### POST /start

#### Description
start the turtlebot on the mission


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|successfully started the mission|No Content|
|**400**|encountered an error in starting the mission|[Response 400](#start-post-response-400)|

<a name="start-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|







