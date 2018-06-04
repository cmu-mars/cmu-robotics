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
used by the TA to indicate to the TH that the test is over.
turtlebot has reached the goal and that the mission has been completed.  note that incomplete missions will result in an error and not use this end point.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**doneparams**  <br>*required*|[doneparams](#done-post-doneparams)|

<a name="done-post-doneparams"></a>
**doneparams**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*required*|final charge measure of the turtlebot. cannot be more than the maximum specified in the response from `/ready`.  <br>**Minimum value** : `0`|integer|
|**message**  <br>*optional*|human-readable text with more information about the end of the test.|string|
|**outcome**  <br>*required*|indicates the reason why the test is over<br><br>  * at-goal - the turtlebot has reached the goal and<br>              completed the mission objectives<br><br>  * out-of-battery - the battery on the turtlebot has run<br>                     out, and cannot be charged, so the<br>                     turtlebot cannot make progress<br><br>  * other-outcome - the test is over for any other<br>                    non-error reason|enum (at-goal, out-of-battery, other-outcome)|
|**sim-time**  <br>*required*|the final internal simulation time  <br>**Minimum value** : `0`|integer|
|**tasks-finished**  <br>*required*|the names of the waypoints that the turtlebot visited in the order that it visited them as well as the x,y coordinates of the robot and simulation time that it arrived there|< [tasks-finished](#done-post-tasks-finished) > array|
|**x**  <br>*required*|final x-coordinate of the turtlebot|number (float)|
|**y**  <br>*required*|final y-coordinate of the turtlebot|number (float)|

<a name="done-post-tasks-finished"></a>
**tasks-finished**

|Name|Description|Schema|
|---|---|---|
|**name**  <br>*optional*|the name of the way point reached (TODO -- this will become an enum when we know all the names of the way points)|string|
|**sim-time**  <br>*optional*|the simulation time when the robot reached this way point|integer|
|**x**  <br>*optional*|the x-coordinate of the robot when it reached this way point|number (float)|
|**y**  <br>*optional*|the y-coordinate of the robot when it reached this way point|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH acknowledges the completion of the mission|No Content|
|**400**|TH encountered an error at the completion of the mission|No Content|


<a name="error-post"></a>
### POST /error

#### Description
used by the TA to indicate to the TH that a non-recoverable error has occurred and the test cannot proceed. the TH will terminate the test upon notification of an error.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**errorparams**  <br>*required*|[errorparams](#error-post-errorparams)|

<a name="error-post-errorparams"></a>
**errorparams**

|Name|Description|Schema|
|---|---|---|
|**error**  <br>*required*|one of a enumerated set of reasons that errors may arise<br><br> * parsing-error - one or more of the function<br>                   descriptions failed to parse<br><br> * learning-error - an error was encountered in learning<br>                    one or more of the hidden functions<br><br> * other-error - an error was encountered that is not<br>                 covered by the other error codees|enum (parsing-error, learning-error, other-error)|
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
|**discharge-budget**  <br>*optional*|if in level c, the maximum number of queries against the target function during learning  <br>**Minimum value** : `2`  <br>**Maximum value** : `1048576`|integer|
|**level**  <br>*required*|the level at which the DAS should operate for this test. as given in the CP definition,<br><br>  * a - no perturbations, no adaptation, no power model<br><br>  * b - perturbations, but no adaptation, no power model<br><br>  * c - perturbations and adaptation, with charge and<br>        discharge power models provided and learned|enum (a, b, c)|
|**power-model**  <br>*optional*|if in level c, the name of the power model from the test data to use for this test. each power model includes at least a function describing how the battery charges, discharges, and a maximum possible charge.  <br>**Minimum value** : `0`  <br>**Maximum value** : `99`|integer|
|**start-loc**  <br>*required*|the name of the start map waypoint. start-loc must not be the same as the first item of `target-locs`.|string|
|**target-locs**  <br>*required*|the names of the waypoints to visit, in the order in which they must be visited. each name must be a valid name of a waypoint on the map. `target-locs` must not be the empty list.  every adjacent pair of elements of `target-locs` must be disequal -- that is to say, it is not permitted to direct the robot to travel to the waypoint where it is already located.|< string > array|


<a name="status-post"></a>
### POST /status

#### Description
used by the TA to periodically indicate its current state to the TH


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**statusparams**  <br>*required*|[statusparams](#status-post-statusparams)|

<a name="status-post-statusparams"></a>
**statusparams**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*required*|current turtlebot battery charge in mWh. cannot be more than the maximum specified in the response from `/ready`.  <br>**Minimum value** : `0`|integer|
|**sim-time**  <br>*required*|the internal simulation time at the time that the status message was sent  <br>**Minimum value** : `0`|integer|
|**status**  <br>*required*|one of the possible status codes<br><br> * learning-started - the learning phase has started<br><br> * learning-done - the learning phase has been<br><br> * adapt-started - the SUT has started adapting and<br>                   cannot be perturbed<br><br> * adapt-done - the SUT has finished adapting<br><br> * charging-started - the turtlebot is currently charging<br><br> * charging-done - the turtlebot has stopped charging<br><br> * at-waypoint - the turtlebot has arrived at one of the<br>   waypoints in the list received from /ready<br><br> * live - the system is ready to recieve perturbs|enum (learning-started, learning-done, adapt-started, adapt-done, charging-started, charging-done, at-waypoint, live)|
|**x**  <br>*required*|current x-coordinate of the turtlebot|number (float)|
|**y**  <br>*required*|current y-coordinate of the turtlebot|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH acknowledges the status message|No Content|
|**400**|TH encountered an error with the status message|No Content|







