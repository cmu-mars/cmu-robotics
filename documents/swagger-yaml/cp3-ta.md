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

<a name="action-das-post"></a>
### POST /action/das

#### Description
enables or disables the DAS


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**das parameters**  <br>*optional*|[das parameters](#action-das-post-das-parameters)|

<a name="action-das-post-das-parameters"></a>
**das parameters**

|Name|Description|Schema|
|---|---|---|
|**enable**  <br>*required*|enable the DAS if the value is `true`, disable it otherwise|boolean|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|sucuessfully changed the DAS state|No Content|
|**400**|encountered an error while changing the DAS state|[Response 400](#action-das-post-response-400)|

<a name="action-das-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|


<a name="action-observe-get"></a>
### GET /action/observe

#### Description
returns observations about the current state of the SUT


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|successfully determined the current state of the SUT|[Response 200](#action-observe-get-response-200)|
|**400**|encountered an error determining the current state of the DAS.|[Response 400](#action-observe-get-response-400)|

<a name="action-observe-get-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*optional*|current charge of the battery, in mWh. TODO -- check the min and max values here  <br>**Minimum value** : `16000`  <br>**Maximum value** : `32000`|integer|
|**kinect_status**  <br>*optional*|the current status of the kinect sensor. "on" means that the sensor is on and being used to gather data about the environment; "off" means that the sensor is off.|enum (on, off)|
|**predicted_arrival**  <br>*optional*|current predicted arrival time, in simulation seconds|integer|
|**sim_time**  <br>*optional*|current simulation time|integer|
|**v**  <br>*optional*|current pitch (TODO) of the turtlebot aspect|number (float)|
|**w**  <br>*optional*|current yaw (TODO) of the turtlebot aspect|number (float)|
|**x**  <br>*optional*|current x-coordinate of the turtlebot position|number (float)|
|**y**  <br>*optional*|current y-coordinate of the turtlebot position|number (float)|

<a name="action-observe-get-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|


<a name="action-start-post"></a>
### POST /action/start

#### Description
start the turtlebot navigating through the selected map


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|successfully started the experiment|No Content|
|**400**|encountered an error trying to start the experiment.|[Response 400](#action-start-post-response-400)|

<a name="action-start-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|


<a name="perturb-battery-post"></a>
### POST /perturb/battery

#### Description
set the level of the battery in a currently running test


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**place obstacle parameters**  <br>*optional*|[place obstacle parameters](#perturb-battery-post-place-obstacle-parameters)|

<a name="perturb-battery-post-place-obstacle-parameters"></a>
**place obstacle parameters**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*required*|the level to which the battery should be set, in mWh. TODO -- are those the right constants? does it matter if battery sets are monotonically decreasing throughout the test? can the TH ever make the battery have more charge all of a sudden?  <br>**Minimum value** : `16000`  <br>**Maximum value** : `32000`|number|


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
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|


<a name="perturb-kinect-post"></a>
### POST /perturb/kinect

#### Description
set the state of the kinect


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**set kinect parameters**  <br>*optional*|[set kinect parameters](#perturb-kinect-post-set-kinect-parameters)|

<a name="perturb-kinect-post-set-kinect-parameters"></a>
**set kinect parameters**

|Name|Description|Schema|
|---|---|---|
|**state**  <br>*required*|the state the kinect should have after this request is processed -- "on" means the kinect is operational and can be used; "off" means it is not operational and cannot be used.|enum (on, off)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the kinect state has been set|[Response 200](#perturb-kinect-post-response-200)|
|**400**|an error was encountered while setting the kinect|[Response 400](#perturb-kinect-post-response-400)|

<a name="perturb-kinect-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**sim_time**  <br>*required*|the simulation time when the kinect was set|integer|

<a name="perturb-kinect-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*required*|human readable info about what went wrong|string|


<a name="perturb-place_obstacle-post"></a>
### POST /perturb/place_obstacle

#### Description
if the test is running, place an obstacle on the map


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**place obstacle parameters**  <br>*optional*|[place obstacle parameters](#perturb-place_obstacle-post-place-obstacle-parameters)|

<a name="perturb-place_obstacle-post-place-obstacle-parameters"></a>
**place obstacle parameters**

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
|**Body**|**place obstacle parameters**  <br>*optional*|[place obstacle parameters](#perturb-remove_obstacle-post-place-obstacle-parameters)|

<a name="perturb-remove_obstacle-post-place-obstacle-parameters"></a>
**place obstacle parameters**

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


<a name="query-initial-get"></a>
### GET /query/initial

#### Description
after ready is reported, the TH will query this end point to get the initial planned path. note that waypoint names are unique per map. the `predicted_arrival_time` is a lower bound on the number of simulation seconds we estimate are needed to traverse the path.


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|computed the initially planned planned path|[Response 200](#query-initial-get-response-200)|
|**400**|encountered an error computing the initially planned path|[Response 400](#query-initial-get-response-400)|

<a name="query-initial-get-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**path**  <br>*required*|the initially planned sequence of waypoints the turtlebot will travese, in order of planned traversal|< string > array|
|**predicted_arrival_time**  <br>*required*|the number of simulation seconds we anticipate traveling the provided path will take|integer|

<a name="query-initial-get-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|







