# cmu mars brass th: phase 2, cp3


<a name="overview"></a>
## Overview

### Version information
*Version* : 0.1


### URI scheme
*Host* : brass-th  
*Schemes* : HTTP




<a name="paths"></a>
## Paths

<a name="done-post"></a>
### POST /done

#### Description
indicates that the test is completed


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#done-post-parameters)|

<a name="done-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**collisions**  <br>*required*|A (possibly empty) list of locations, times, and speeds of any collisions of the robot|< [CollisionData](#collisiondata) > array|
|**final-charge**  <br>*required*|the charge left in the battery when the test ended  <br>**Minimum value** : `0`|integer|
|**final-sim-time**  <br>*required*|the simulation time when the mission finished  <br>**Minimum value** : `0`|integer|
|**final-utility**  <br>*required*|the utility achieved at the end of the mission, in the range [0,1]  <br>**Minimum value** : `0`  <br>**Maximum value** : `1`|number (float)|
|**final-x**  <br>*required*|the x coordinate of the robot position when the test ended|number (float)|
|**final-y**  <br>*required*|the y coordinate of the robot position when the test ended|number (float)|
|**num-adaptations**  <br>*required*|the number of times that the robot adapted (0 if there is no DAS)  <br>**Minimum value** : `0`|integer|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the done message from the SUT and is shutting down the test|No Content|
|**400**|the TH has encountered an error in processing the done message of the SUT and is shutting down the test|No Content|


<a name="error-post"></a>
### POST /error

#### Description
indicates that the SUT has encountered an error in configuration data, parameters, system start, or any other less specified problem


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#error-post-parameters)|

<a name="error-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human readable text describing the error, if possible. (TODO: add error codes here as we discover them)|string|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges that the SUT has encountered an error and will shut down the test|No Content|
|**400**|the TH has encountered an error in processing the error from the SUT and will also shut down the test|No Content|


<a name="ready-post"></a>
### POST /ready

#### Description
indicates that the SUT is ready to recieve configuration data to continue start up


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH is ready to produce configuration data|[Response 200](#ready-post-response-200)|
|**400**|TH encountered an error producing configuration data|No Content|

<a name="ready-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**start-configuration**  <br>*optional*|the label of the starting configuration for the robot|enum (amcl-kinect, amcl-lidar, mprt-kinect, mprt-lidar, aruco-camera)|
|**start-loc**  <br>*optional*|the name of the start map waypoint. must be a valid way point name from the map data. must not be equal to `target-loc`.|string|
|**target-loc**  <br>*optional*|the name of the goal map waypoint. must be a valid way point name from the map data. must not be equal to `start-loc`.|string|
|**use-adaptation**  <br>*optional*|if `true`, then the DAS will use adapative behaiviours; if `false` then the DAS will not use adaptive behaiviours|boolean|
|**utility-function**  <br>*optional*|the utility function to use for evaluating mission quality|enum (favor-timeliness, favor-safety, favor-efficiency)|


<a name="status-post"></a>
### POST /status

#### Description
indicate important state changes in the SUT to the TH. posted periodically as the described events occur.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#status-post-parameters)|

<a name="status-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**config**  <br>*optional*|list of currently active nodes. This will not be sent with `status == live`.|< enum (amcl-kinect, amcl-lidar, mrpt-kinect, mrpt-lidar, aruco-camera) > array|
|**message**  <br>*optional*|human readable text describing the status, if any|string|
|**plan**  <br>*optional*|list of waypoints the current plan tends to visit, in order. This will not be sent with `status == live`.|< string > array|
|**sensors**  <br>*optional*|list of currently active sensors, in order. This will not be sent with `status == live`.|< enum (kinect, lidar, camera, headlamp) > array|
|**sim-time**  <br>*required*|the simulation time the status message was produced  <br>**Minimum value** : `0`|integer|
|**status**  <br>*required*|one of a enumerated set of statuses to report, arise, as follows:<br>  * `live`, the SUT has processed the configuration data<br>     and is ready for initial perturbations (if any) and the<br>     start of the test<br><br>  * `mission-running`, the SUT has processed the initial<br>     perturbations after receiving `/start`, possibly<br>     adapted, and the robot is now actually moving along<br>     its path. it is an error to send any perturbation to<br>     the SUT between sending a message to `/start` and<br>     receiving this status.<br><br>  * `adapting`, the SUT has detected a condition that<br>     requires adaptation and the SUT is adapting. it is<br>     an error to send any perturbation to the SUT after<br>     this message is sent to the TH until the TH gets a<br>     status message with `adapted`.<br><br>  * `adapted`, the SUT has finished adapting after<br>     observing a need to. this means that the robot is<br>     moving along its plan again and it is no longer an<br>     error to send perturbations. if this is the status<br>     code of the message, the fields `plan`, `config` and<br>     `sensors` will also be present, to describe the new<br>     state of the robot.|enum (live, mission-running, adapting, adapted)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the status of the SUT|No Content|
|**400**|the TH has encountered an error in processing the status of the SUT|No Content|




<a name="definitions"></a>
## Definitions

<a name="collisiondata"></a>
### CollisionData

|Name|Description|Schema|
|---|---|---|
|**robot-speed**  <br>*required*|The speed of the robot (m/s) at the time the collision occured|number (float)|
|**robot-x**  <br>*required*|The x location of the robot center when collision occured|number (float)|
|**robot-y**  <br>*required*|The y lcoation of the robot center when collision occured|number (float)|
|**sim-time**  <br>*required*|The (simulation) time at which the collision occurred (seconds from start of simulator)|integer|





