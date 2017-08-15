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
|**Body**|**error info**  <br>*optional*|[error info](#done-post-error-info)|

<a name="done-post-error-info"></a>
**error info**

|Name|Description|Schema|
|---|---|---|
|**REASON**  <br>*required*|TODO, make this an enum?|string|
|**TARGET**  <br>*required*|TODO|string|
|**sim_time**  <br>*required*|the simulation time when the mission finished|integer|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the done message from the SUT and is shutting down the test|No Content|
|**400**|the TH has encountered an error in processing the done message of the SUT|No Content|


<a name="error-post"></a>
### POST /error

#### Description
indicates that the SUT has encountered an error in configuration data, parameters, system start, or any other less specified problem


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**error info**  <br>*optional*|[error info](#error-post-error-info)|

<a name="error-post-error-info"></a>
**error info**

|Name|Description|Schema|
|---|---|---|
|**ERROR**  <br>*required*|one of a enumerated set of reasons that errors may arise|enum (TEST_DATA_FILE_ERROR, TEST_DATA_FORMAT_ERROR, DAS_LOG_FILE_ERROR, DAS_OTHER_ERROR)|
|**MESSAGE**  <br>*required*|human readable text describing the error|string|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges that the SUT has encountered an error and will shut down the test|No Content|
|**400**|the TH has encountered an error in processing the error from the SUt and will also shut down the test|No Content|


<a name="ready-post"></a>
### POST /ready

#### Description
indicate to the TH that the TA is ready to recieve configuration data and continue starting up the DAS


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH is ready to produce configuration data|[Response 200](#ready-post-response-200)|
|**400**|TH encountered an error producing configuration data|No Content|

<a name="ready-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**map_to_use**  <br>*optional*|the name of the map to use for this test, must be from the list of agreed upon map names|string (MapMnemonic)|
|**start_loc**  <br>*optional*|the name of the start map waypoint. must be a valid way point for the map given in `map_to_use`.|string|
|**target_loc**  <br>*optional*|the name of the goal map waypoint|string|
|**use_adaptation**  <br>*optional*|if `true`, then the DAS will use adapative behaiviours; if `false` then the DAS will not sure adaptive behaiviours|boolean|


<a name="start-post"></a>
### POST /start

#### Description
indicates that the SUT is ready to recieve perturbations and that the test has started


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges that the SUT has started|No Content|
|**400**|the TH has encountered an error|No Content|


<a name="status-post"></a>
### POST /status

#### Description
indicate important state changes in the SUT to the TH. posted periodically as the described events occur.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**error info**  <br>*optional*|[error info](#status-post-error-info)|

<a name="status-post-error-info"></a>
**error info**

|Name|Description|Schema|
|---|---|---|
|**ERROR**  <br>*required*|one of a enumerated set of reasons that errors may arise, described as follows<br> * BOOTING - TODO<br> * BOOTED - TODO<br> * ONLINE - TODO<br> * PERTURBATION_DETECTED - TODO<br> * MISSION_SUSPENDED - TODO<br> * MISSION_RESUMED - TODO<br> * MISSION_HALTED - TODO<br> * MISSION_ABORTED - TODO<br> * ADAPTATION_INITIATED - TODO<br> * ADAPTATION_COMPLETED - TODO<br> * ADAPTATION_STOPPED - TODO<br> * TEST_ERROR - TODO|enum (BOOTING, BOOTED, ONLINE, PERTURBATION_DETECTED, MISSION_SUSPENDED, MISSION_RESUMED, MISSION_HALTED, MISSION_ABORTED, ADAPTATION_INITIATED, ADAPTATION_COMPLETED, ADAPTATION_STOPPED, TEST_ERROR)|
|**MESSAGE**  <br>*required*|human readable text describing the status|string|
|**sim_time**  <br>*required*|the time inside the simulation when the status message was produced|integer|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the status of the SUT|No Content|
|**400**|the TH has encountered an error in processing the status of the SUT|No Content|






