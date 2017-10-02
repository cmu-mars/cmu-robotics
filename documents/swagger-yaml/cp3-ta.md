# cmu mars brass ta: phase 2, cp3


<a name="overview"></a>
## Overview

### Version information
*Version* : 0.1


### URI scheme
*Host* : brass-ta  
*Schemes* : HTTP




<a name="paths"></a>
## Paths

<a name="perturb-light-post"></a>
### POST /perturb/light

#### Description
change the state of a light in the environment


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#perturb-light-post-parameters)|

<a name="perturb-light-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**id**  <br>*required*|the element of LIGHTSET whose state will be set|string|
|**state**  <br>*required*|upon response, the named light will be on if `true` and `off` otherwise|boolean|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the named light has been set|[Response 200](#perturb-light-post-response-200)|
|**400**|an error was encountered while setting the light|[Response 400](#perturb-light-post-response-400)|

<a name="perturb-light-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**sim-time**  <br>*required*|the simulation time when the light was set|integer|

<a name="perturb-light-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|
|**sim-time**  <br>*required*|the simulation time when the error occurred|integer|


<a name="perturb-nodefail-post"></a>
### POST /perturb/nodefail

#### Description
cause one of the software nodes to fail


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#perturb-nodefail-post-parameters)|

<a name="perturb-nodefail-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**id**  <br>*required*|cause the named node to fail|enum (movebase, amcl, mrpt, cb-base)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the node has failed|[Response 200](#perturb-nodefail-post-response-200)|
|**400**|an error was encountered while causing the node to fail|[Response 400](#perturb-nodefail-post-response-400)|

<a name="perturb-nodefail-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**sim-time**  <br>*required*|the simulation time when the node failed|integer|

<a name="perturb-nodefail-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|
|**sim-time**  <br>*required*|the simulation time when the error occurred|integer|


<a name="perturb-sensor-post"></a>
### POST /perturb/sensor

#### Description
change the state of one of the sensors on the robot


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#perturb-sensor-post-parameters)|

<a name="perturb-sensor-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**id**  <br>*required*|which sensor of SENSORSET to set|enum (kinect-ir, lidar, kinect-all)|
|**state**  <br>*required*|upon response, the named sensor will be on if `true` and `off` otherwise|boolean|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the named sensor has been set|[Response 200](#perturb-sensor-post-response-200)|
|**400**|an error was encountered while setting the sensor|[Response 400](#perturb-sensor-post-response-400)|

<a name="perturb-sensor-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**sim-time**  <br>*required*|the simulation time when the sensor was set|integer|

<a name="perturb-sensor-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|
|**sim-time**  <br>*required*|the simulation time when the error occurred|integer|


<a name="start-post"></a>
### POST /start

#### Description
start the turtlebot navigating through the map


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|SUT has begin starting the mission|No Content|
|**400**|SUT encountered an error trying to start the mission|[Response 400](#start-post-response-400)|

<a name="start-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|







