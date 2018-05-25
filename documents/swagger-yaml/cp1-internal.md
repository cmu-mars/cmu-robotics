# CP1 Internal Status API


<a name="overview"></a>
## Overview

### Version information
*Version* : 0.1


### URI scheme
*Host* : brass-ta  
*Schemes* : HTTP




<a name="paths"></a>
## Paths

<a name="internal-status-post"></a>
### POST /internal-status

#### Description
reports any internal status (including the error that may occured) from the backend that might be sent to the TA for internal bookeeping or forwarding to the TH


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**CP1InternalStatus**  <br>*required*|[CP1InternalStatus](#internal-status-post-cp1internalstatus)|

<a name="internal-status-post-cp1internalstatus"></a>
**CP1InternalStatus**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*required*|human readable text describing the status, if any|string|
|**sim-time**  <br>*optional*|the simulation time the status message was produced, for some siutations like learning-* we do not have simulation time because the simulation has not been started  <br>**Minimum value** : `0`|integer|
|**status**  <br>*required*|one of the possible status codes * learning-started - the learning phase has started * learning-done - the learning phase has been * adapt-started - the SUT has started adapting and cannot be perturbed * adapt-done - the SUT has finished adapting * charging-started - the turtlebot is currently charging * charging-done - the turtlebot has stopped charging * parsing-error - one or more of the function descriptions failed to parse * learning-error - an error was encountered in learning one or more of the hidden functions * other-error - an error was encountered that is not covered by the other error codees|enum (learning-started, learning-done, adapt-started, adapt-done, charging-started, charging-done, parsing-error, learning-error, other-error, RAINBOW_READY, MISSION_SUCCEEDED, MISSION_FAILED, ADAPTING, ADAPTED, ADAPTED_FAILED)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TA acknowledges the status of the SUT|No Content|
|**400**|TA has encountered an error in processing the status of the SUT.|No Content|







