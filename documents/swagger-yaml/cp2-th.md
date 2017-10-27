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
|**Body**|**Parameters**  <br>*required*|[Parameters](#done-post-parameters)|

<a name="done-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**log**  <br>*required*|A list containing details of each of the attempted repairs.|< [CandidateAdaptation](#candidateadaptation) > array|
|**num-attempts**  <br>*required*|The number of code adaptations attempted.  <br>**Minimum value** : `0`|integer|
|**outcome**  <br>*required*|A short description of the success of the repair process. A complete repair is one which fully restores the intent of the system (i.e., system degradation is reduced to zero). A partial repair reduces the degradation of the system from its initial value to a value greater than zero. If the level of degradation remains unchanged, no (partial) repair has been found.|enum (CompleteRepair, PartialRepair, NoRepair)|
|**pareto-set**  <br>*required*|A list containing details of all adaptations within the pareto set.|< [CandidateAdaptation](#candidateadaptation) > array|
|**running-time**  <br>*required*|The number of minutes taken to complete the repair process.  <br>**Minimum value** : `0`|number (float)|


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
system under test.
\
**Error codes**:
  * *NeutralPerturbation:* One of the perturbations for the test scenario has no
    effect on the outcome of the test suite, and as such, it
    does not consistitute a fault.
  * *PerturbationFailedToCompile:* The perturbed version of the system
    failed to compile.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#error-post-parameters)|

<a name="error-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**err-code**  <br>*required*|Used to indicate the type of error that has occurred.|enum (NeutralPerturbation, PertubationFailedToCompile)|
|**err-description**  <br>*optional*|An optional description of the error.|string|


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
|**200**|the TH acknowledges the ready message|[Response 200](#ready-post-response-200)|
|**400**|the TH has itself encontered an error processing the ready message|No Content|

<a name="ready-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**docker-ip**  <br>*required*|the IP address to connect to in order to allocate new worker containers|string (ip-address)|


<a name="status-post"></a>
### POST /status

#### Description
Used to inform the test harness that a new adaptation has been added to the Pareto set (i.e., a new "best" adaptation has been found).


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#status-post-parameters)|

<a name="status-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**adaptation**  <br>*required*|The candidate adaptation that was added to the Pareto set.|[CandidateAdaptation](#candidateadaptation)|
|**pareto-set**  <br>*required*|The current contents of the Pareto-set of adaptations.|< [CandidateAdaptation](#candidateadaptation) > array|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the status|No Content|
|**400**|the TH has itself encontered an error processing the status|No Content|




<a name="definitions"></a>
## Definitions

<a name="candidateadaptation"></a>
### CandidateAdaptation

|Name|Description|Schema|
|---|---|---|
|**compilation-outcome**  <br>*required*|A description of the outcome of attempting to compile this adaptation.|[CompilationOutcome](#compilationoutcome)|
|**degradation**  <br>*required*|A description of the level of degradation that was observed when this adaptation was applied.|[Degradation](#degradation)|
|**diff**  <br>*required*|A description of the change to the code, given in the form of a diff.|string|
|**test-outcomes**  <br>*required*|A summary of the outcomes for each of the test cases that this adaptation was evaluated against.|< [TestOutcome](#testoutcome) > array|


<a name="compilationoutcome"></a>
### CompilationOutcome

|Name|Description|Schema|
|---|---|---|
|**successful**  <br>*required*|A flag indicating whether the compilation of this adaptation was successful or not.|boolean|
|**time-taken**  <br>*required*|The number of seconds taken to compile this adaptation.  <br>**Minimum value** : `0`|number (float)|


<a name="degradation"></a>
### Degradation
*Type* : object


<a name="testoutcome"></a>
### TestOutcome

|Name|Description|Schema|
|---|---|---|
|**crashed**  <br>*optional*|A flag indicating whether or not the system crashed during execution of the test.|boolean|
|**qos**  <br>*optional*|A summary of the quality of service that was observed during the execution of the test.|[TestQoS](#testqos)|
|**test-id**  <br>*required*|A unique identifier for the test to which this outcome belongs.|string|
|**time-taken**  <br>*required*|The number of seconds taken to complete the test.  <br>**Minimum value** : `0`|number (float)|
|**timed-out**  <br>*required*|A flag indicating whether or not the test timed out during execution.|boolean|


<a name="testqos"></a>
### TestQoS

|Name|Description|Schema|
|---|---|---|
|**collisions**  <br>*required*|A measure of service quality with respect to the number of collisions.|object|
|**duration**  <br>*required*|A measure of service quality with respect to time taken to complete the test.|object|
|**proximity**  <br>*required*|A measure of service quality with respect to proximity to the goal.|object|





