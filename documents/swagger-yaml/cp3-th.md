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
|**arrival-predictions**  <br>*required*|all the predicted arrival times made by the SUT during<br>     the test, in the order they were made.|< integer > array|
|**final-charge**  <br>*required*|the charge left in the battery when the test ended|integer|
|**final-sim-time**  <br>*required*|the simulation time when the mission finished  <br>**Minimum value** : `0`|integer|
|**final-x**  <br>*required*|the x coordinate of the robot position when the test ended|number (float)|
|**final-y**  <br>*required*|the y coordinate of the robot position when the test ended|number (float)|


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
|**error**  <br>*required*|one of a enumerated set of reasons that errors may arise|enum (??? todo ???)|
|**message**  <br>*optional*|human readable text describing the error, if possible|string|


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
|**start-loc**  <br>*optional*|the name of the start map waypoint. must be a valid way point name from the map data.|string|
|**target-loc**  <br>*optional*|the name of the goal map waypoint. must be a valid way point name from the map data.|string|
|**use-adaptation**  <br>*optional*|if `true`, then the DAS will use adapative behaiviours; if `false` then the DAS will not use adaptive behaiviours|boolean|


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
|**message**  <br>*optional*|human readable text describing the status, if any|string|
|**sim-time**  <br>*required*|the simulation time the status message was produced  <br>**Minimum value** : `0`|integer|
|**status**  <br>*required*|one of a enumerated set of reasons that errors may arise, as follows:<br>  * `live`, the SUT has processed the configuration data<br>     and is ready for initial perturbations (if any) and the<br>     start of the test<br><br>  * `mission-running`, the SUT has processed the initial<br>     perturbations after receiving `/start`, possibly<br>     adapted, and the robot is now actually moving along<br>     its path. it is an error to send any perturbation to<br>     the SUT between sending a message to `/start` and<br>     receiving this status.<br><br>  * `adapting`, the SUT has detected a condition that<br>     requires adaptation and the SUT is adapting. it is<br>     an error to send any perturbation to the SUT after<br>     this message is sent to the TH until the TH gets a<br>     status message with `adapted`.<br><br>  * `adapted`, the SUT has finished adapting after<br>     observing a need to. this means that the robot is<br>     moving along its plan again and it is no longer an<br>     error to send perturbations. if this is the status<br>     code of the message, the fields `plan`, `config` and<br>     `sensors` will also be present, to describe the new<br>     state of the robot.|enum (live, mission-running, adapting, adapted)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the status of the SUT|No Content|
|**400**|the TH has encountered an error in processing the status of the SUT|No Content|







