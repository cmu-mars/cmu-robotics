# CP3 Internal Status API


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
reports any internal status that might be sent to the TA for internal bookeeping or forwarding to the TH


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**CP3InternalStatus**  <br>*required*|[CP3InternalStatus](#internal-status-post-cp3internalstatus)|

<a name="internal-status-post-cp3internalstatus"></a>
**CP3InternalStatus**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*required*|human readable text describing the status, if any|string|
|**sim-time**  <br>*required*|the simulation time the status message was produced  <br>**Minimum value** : `0`|integer|
|**status**  <br>*required*|one of a enumerated set of statuses to report, arise, as follows:<br>  * `RAINBOW_READY`, Rainbow is up and ready for the mission<br>    to start<br><br>  * `MISSION_SUCCEEDED`, According to the SUT, the mission has<br>    successfully completed. This will only be reported in the<br>    Challenge case.<br><br>  * `MISSION_FAILED`, According to the SUT, the mission has<br>    failed. This will only be reported in the Challenge case.<br><br>  * `ADAPTING`, the SUT has started adapting, which means looking<br>    for a new plan.<br><br>  * `ADAPTED`, the SUT has found a new plan and it has been issued to<br>    the system.<br><br>  * `ADAPTED_FAILED`, the plan failed to be executed on the system<br><br>  * `FINAL_UTILITY` the final utility achieved by<br>    the system. Here the message will simply be a<br>    floating point number printed as a string<br><br>  * `PLAN` the new list of waypoints being visted by<br>    the system because of replanning. The message<br>    will be a stringified json array of strings.|enum (RAINBOW_READY, MISSION_SUCCEEDED, MISSION_FAILED, ADAPTING, ADAPTED, ADAPTED_FAILED, FINAL_UTILITY, PLAN)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the status of the SUT|No Content|
|**400**|the TH has encountered an error in processing the status of the SUT|No Content|







