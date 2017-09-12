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

<a name="action-observe-get"></a>
### GET /action/observe

#### Description
the current state of the SUT


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|successfully determined the current state of the SUT|[Response 200](#action-observe-get-response-200)|
|**400**|encountered an error determining the current state of the SUT.|[Response 400](#action-observe-get-response-400)|

<a name="action-observe-get-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*optional*|current charge of the battery, in mWh. TODO -- check the min and max values here  <br>**Minimum value** : `16000`  <br>**Maximum value** : `32000`|integer|
|**kinect_status**  <br>*optional*|the current status of the kinect sensor. "on" means that the sensor is on and being used to gather data about the environment; "off" means that the sensor is off.|enum (on, off)|
|**predicted_arrival**  <br>*optional*|current predicted arrival time, in simulation seconds|integer|
|**sim_time**  <br>*optional*|current simulation time  <br>**Minimum value** : `0`|integer|
|**v**  <br>*optional*|current velocity of the turtlebot|number (float)|
|**w**  <br>*optional*|current yaw of the turtlebot|number (float)|
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
set the level of the battery. using this end point before getting a `live` status message is an error.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#perturb-battery-post-parameters)|

<a name="perturb-battery-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*required*|the level to which the battery should be set, in mWh. TODO -- are those the right constants? does it matter if battery sets are monotonically decreasing throughout the test? can the TH ever make the battery have more charge all of a sudden?  <br>**Minimum value** : `16000`  <br>**Maximum value** : `32000`|number|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the battery has been set to the requested level|[Response 200](#perturb-battery-post-response-200)|
|**400**|an error was encountered while setting the battery level|[Response 400](#perturb-battery-post-response-400)|

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


<a name="perturb-place_obstacle-post"></a>
### POST /perturb/place_obstacle

#### Description
place an obstacle on the map.  using this end point before getting a `live` status message is an error.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**place obstacle parameters**  <br>*required*|[place obstacle parameters](#perturb-place_obstacle-post-place-obstacle-parameters)|

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
remove a previously placed obstacle from the map. using this end point before getting a `live` status message is an error. sending any obstacle ID string that was not recived from a previous use of the `place_obstacle` end point is an error.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#perturb-remove_obstacle-post-parameters)|

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


<a name="perturb-robot-post"></a>
### POST /perturb/robot

#### Description
introduce an error in the robot software. using this end point before getting a `live` status message is an error.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**set a part of the robot to fail**  <br>*required*|[set a part of the robot to fail](#perturb-robot-post-set-a-part-of-the-robot-to-fail)|

<a name="perturb-robot-post-set-a-part-of-the-robot-to-fail"></a>
**set a part of the robot to fail**

|Name|Description|Schema|
|---|---|---|
|**item**  <br>*required*||enum (kinect, ultrasound, node1, node2)|
|**state**  <br>*required*|the state the kinect should have after this request is processed -- "on" means the kinect is operational and can be used; "off" means it is not operational and cannot be used.|enum (on, off, failing)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the error has been introduced into the robot software|[Response 200](#perturb-robot-post-response-200)|
|**400**|an error was encountered while introducing an error into the robot software|[Response 400](#perturb-robot-post-response-400)|

<a name="perturb-robot-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**sim_time**  <br>*required*|the simulation time when the error was introduced|integer|

<a name="perturb-robot-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*required*|human readable info about what went wrong|string|


<a name="query-initial-get"></a>
### GET /query/initial

#### Description
after ready is reported, the TH will query this end point to get the initial planned path. note that waypoint names are unique per map.


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|computed the initially planned path|[Response 200](#query-initial-get-response-200)|
|**400**|encountered an error computing the initially planned path|[Response 400](#query-initial-get-response-400)|

<a name="query-initial-get-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**path**  <br>*required*|the initially planned sequence of waypoints the turtlebot will traverse, in order of planned traversal|< string (waypoint-names) > array|
|**predicted_arrival_time**  <br>*required*|a lower bound estimate of the number of simulation seconds needed for the robot to traverse the planned path  <br>**Minimum value** : `0`|integer|

<a name="query-initial-get-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable information about the error, if any can be provided|string|







