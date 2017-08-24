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
|**charge**  <br>*required*|final charge measure of the turtlebot TODO - check ranges  <br>**Minimum value** : `16000`  <br>**Maximum value** : `32000`|integer|
|**message**  <br>*required*|TODO human readable something or other|string|
|**predicted_arrival**  <br>*required*|final best prediction of arrival time, in simulation time  <br>**Minimum value** : `0`|integer|
|**sim_time**  <br>*required*|the final internal simulation time  <br>**Minimum value** : `0`|integer|
|**v**  <br>*required*|final ?TODO? of the turtlebot|number (float)|
|**w**  <br>*required*|final ?TODO? of the turtlebot|number (float)|
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
|**ERROR**  <br>*required*|one of a enumerated set of reasons that errors may arise<br>  * TEST_DATA_FILE_ERROR - TODO<br>  * TEST_DATA_FORMAT_ERROR - TODO<br>  * DAS_LOG_FILE_ERROR - TODO<br>  * DAS_OTHER_ERROR - TODO<br>  * PARSING_ERROR - TODO<br>  * LEARNING_ERROR - TODO|enum (TEST_DATA_FILE_ERROR, TEST_DATA_FORMAT_ERROR, DAS_LOG_FILE_ERROR, DAS_OTHER_ERROR, PARSING_ERROR, LEARNING_ERROR)|
|**MESSAGE**  <br>*required*|human readable text describing the error|string|


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
|**charge_budget**  <br>*optional*|the maximum number of queries against the target recharging function during learning|integer|
|**charge_function**  <br>*optional*|a description of the function dictating the recharging of the battery, which is what we will learn. TODO - this can't be a string|string (function_spec)|
|**discharge_budget**  <br>*optional*|the maximum number of queries against the target function during learning|integer|
|**discharge_function**  <br>*optional*|a description of the function dictating the discharge of the battery, which is what we will learn. TODO - this can't be a string|string (function_spec)|
|**level**  <br>*optional*|the level at which the DAS should operate for this test<br>  * pert - TODO<br>  * no_adaptation - TODO|enum (pert, no_adaptation)|
|**model**  <br>*optional*|the type of power model to use for this test<br>  * no_pm - TODO<br>  * predefined_pm - TODO<br>  * learned_pm - TODO|enum (no_pm, predefined_pm, learned_pm)|
|**start_loc**  <br>*optional*|the name of the start map waypoint|string|
|**target_locs**  <br>*optional*|the names of the waypoints to visit, in the order in which they must be visited|< string > array|


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
|**STATUS**  <br>*required*|one of the possible statuses<br> * BOOTING - TODO<br> * BOOTED - TODO<br> * ONLINE - TODO<br> * OFFLINE - TODO<br> * PERTURBATION_DETECTED - TODO<br> * MISSION_SUSPENDED - TODO<br> * MISSION_RESUMED - TODO<br> * MISSION_HALTED - TODO<br> * MISSION_ABORTED - TODO<br> * ADAPTATION_INITIATED - TODO<br> * ADAPTATION_COMPLETED - TODO<br> * ADAPTATION_STOPPED - TODO<br> * TEST_ERROR - TODO<br> * LEARNING_STARTED - TODO<br> * LEARNING_DONE - TODO|enum (BOOTING, BOOTED, ONLINE, OFFLINE, PERTURBATION_DETECTED, MISSION_SUSPENDED, MISSION_RESUMED, MISSION_HALTED, MISSION_ABORTED, ADAPTATION_INITIATED, ADAPTATION_COMPLETED, ADAPTATION_STOPPED, TEST_ERROR, LEARNING_STARTED, LEARNING_DONE)|
|**charge**  <br>*required*|current turtlebot battery charge in mWh TODO - are these mins and maxes right?  <br>**Minimum value** : `16000`  <br>**Maximum value** : `32000`|integer|
|**predicted_arrival**  <br>*required*|current best prediction of arrival time, in simulation time  <br>**Minimum value** : `0`|integer|
|**sim_time**  <br>*required*|the internal simulation time at the time that the status message was sent  <br>**Minimum value** : `0`|integer|
|**v**  <br>*required*|current ? TODO ? of the turtlebot|number (float)|
|**w**  <br>*required*|current ? TODO? of the turtlebot|number (float)|
|**x**  <br>*required*|current x-coordinate of the turtlebot|number (float)|
|**y**  <br>*required*|current y-coordinate of the turtlebot|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH acknowledges the status message|No Content|
|**400**|TH encountered an error with the status message|No Content|







