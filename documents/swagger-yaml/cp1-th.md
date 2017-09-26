# cmu mars brass th: phase 2, cp1


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
used by the TA to indicate to the TH that the turtlebot has reached the goal and that the mission has been completed. note that incomplete missions will result in an error and not use this end point.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#done-post-parameters)|

<a name="done-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*required*|final charge measure of the turtlebot. cannot be more than the maximum specified in the response from `/ready`.  <br>**Minimum value** : `0`|integer|
|**predicted_arrival**  <br>*required*|final best prediction of arrival time, in simulation time  <br>**Minimum value** : `0`|integer|
|**sim_time**  <br>*required*|the final internal simulation time  <br>**Minimum value** : `0`|integer|
|**v**  <br>*required*|final velocity of the turtlebot|number (float)|
|**w**  <br>*required*|final yaw of the turtlebot|number (float)|
|**x**  <br>*required*|final x-coordinate of the turtlebot|number (float)|
|**y**  <br>*required*|final y-coordinate of the turtlebot|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH acknowledges the completion of the mission|No Content|
|**400**|TH encountered an error at the completion of the mission|No Content|


<a name="error-post"></a>
### POST /error

#### Description
used by the TA to indicate to the TH that an error has occurred in the start up or learning process. the TH will terminate the test upon notification of an error


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#error-post-parameters)|

<a name="error-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**error**  <br>*required*|one of a enumerated set of reasons that errors may arise<br> * parsing-error - one or more of the function<br>                   descriptions failed to parse<br> * learning-error - an error was encountered in learning<br>                    one or more of the hidden functions<br> * other-error - an error was encountered that is not<br>                 covered by the other error codees|enum (parsing-error, learning-error, other-error)|
|**message**  <br>*optional*|human readable text describing the error, if available|string|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH acknowledges the error and is shutting down the test|No Content|


<a name="ready-post"></a>
### POST /ready

#### Description
indicate to the TH that the TA is ready to recieve configuration data to continue starting up the DAS


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH is ready to produce configuration data|[Response 200](#ready-post-response-200)|
|**400**|TH encountered an error producing configuration data|No Content|

<a name="ready-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**charge-budget**  <br>*optional*|if in level d, the maximum number of queries against the target recharging function during learning|integer|
|**charge-function**  <br>*optional*|if in level d, a description of the function dictating the recharging of the battery, which is what we will learn.|string (function-spec)|
|**discharge-budget**  <br>*optional*|if in level d, the maximum number of queries against the target function during learning|integer|
|**discharge-function**  <br>*optional*|if in level d, a description of the function dictating the discharge of the battery, which is what we will learn.|string (function-spec)|
|**level**  <br>*required*|the level at which the DAS should operate for this test.<br>as given in the CP definition,<br><br>  * a - no perturbations, no adaptation, no power model<br>  * b - perturbations, but no adaptation, no power model<br>  * c - perturbations and adaptation, but a static power<br>        model for discharge/charge, while planner uses a<br>        different static power model<br>  * d - perturbations and adaptation, with charge and<br>        discharge power models provided and learned|enum (a, b, c, d)|
|**max-charge**  <br>*optional*|if in level d, the maximum charge the battery can hold, in mWh. implicitly, all batteries have a minimum possible charge of 0 mWh  <br>**Minimum value** : `0`|integer|
|**start-loc**  <br>*required*|the name of the start map waypoint. start-loc must not be the same as the first item of `target-locs`.|string|
|**target-locs**  <br>*required*|the names of the waypoints to visit, in the order in which they must be visited. each name must be a valid name of a waypoint on the map. `target-locs` must not be the empty list. every adjacent pair of elements of `target-locs` must be disequal -- that is to say, it is not permitted to direct the robot to go travel to the waypoint where it is already located.|< string > array|


<a name="status-post"></a>
### POST /status

#### Description
used by the TA to periodically indicate its current state to the TH


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#status-post-parameters)|

<a name="status-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*required*|current turtlebot battery charge in mWh. cannot be more than the maximum specified in the response from `/ready`.  <br>**Minimum value** : `0`|integer|
|**predicted_arrival**  <br>*required*|current best prediction of arrival time, in simulation time  <br>**Minimum value** : `0`|integer|
|**sim_time**  <br>*required*|the internal simulation time at the time that the status message was sent  <br>**Minimum value** : `0`|integer|
|**status**  <br>*required*|one of the possible status codes<br> * learning-started - the learning phase has started<br> * learning-done - the learning phase has been<br> * adapt-started - the SUT has started adapting and<br>                   cannot be perturbed<br> * adapt-done - the SUT has finished adapting<br> * charging-started - the turtlebot is currently charging<br> * charging-done - the turtlebot has stopped charging|enum (learning-started, learning-done, adapt-started, adapt-done, charging-started, charging-done)|
|**v**  <br>*required*|current velocity of the turtlebot|number (float)|
|**w**  <br>*required*|current yaw of the turtlebot|number (float)|
|**x**  <br>*required*|current x-coordinate of the turtlebot|number (float)|
|**y**  <br>*required*|current y-coordinate of the turtlebot|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH acknowledges the status message|No Content|
|**400**|TH encountered an error with the status message|No Content|







