# cmu mars brass th: phase 2, cp2


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
Used to indicate that evaluation of the test scenario has been completed. A summary of the results of the test scenario are provided as a JSON object. The entire summary is contained within `SutFinishedStatus`, as specified by the Lincoln Labs API.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#done-post-parameters)|

<a name="done-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**log**  <br>*required*|A list containing details of each of the attempted repairs.|< [CandidateAdaptation](#candidateadaptation) > array|
|**num-attempts**  <br>*required*|The number of code adaptations attempted.|integer|
|**outcome**  <br>*required*|A short description of the success of the repair process TODO -- in the md, Chris had this as an enum but without values listed|string|
|**pareto-set**  <br>*required*|A list containing details of all adaptations within the pareto set.|< [CandidateAdaptation](#candidateadaptation) > array|
|**running-time**  <br>*required*|The number of minutes taken to complete the repair process.|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the done message|No Content|
|**400**|the TH has itself encontered an error processing the done message|No Content|


<a name="error-post"></a>
### POST /error

#### Description
Used to indicate that an error has occurred during the preparation
or evaluation of a test scenario, or during the start-up of the
system under test. The body of this method contains a single
property, `ErrorMsg`, containing a JSON-based description of the
error. A description of the errors produced by SUT is given
below.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#error-post-parameters)|

<a name="error-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**err-code**  <br>*required*|One of the perturbations for the test scenario has no effect on the outcome of the test suite, and as such, it does not consistitute a fault<br>TODO -- add more error codes with more description of possible bad states|enum (NeutralPerturbation)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the error|No Content|
|**400**|the TH has itself encountered an error processing the error|No Content|


<a name="ready-post"></a>
### POST /ready

#### Description
Used to indicate that the SUT is ready and that testing may begin.


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the ready message|No Content|
|**400**|the TH has itself encontered an error processing the ready message|No Content|


<a name="status-post"></a>
### POST /status

#### Description
Used to inform the test harness that a new adaptation has been added to the Pareto set (i.e., a new "best" adaptation has been found). TODO -- add parameters here?


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the status|No Content|
|**400**|the TH has itself encontered an error processing the status|No Content|




<a name="definitions"></a>
## Definitions

<a name="candidateadaptation"></a>
### CandidateAdaptation
*Type* : object





