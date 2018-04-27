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
|**outcome**  <br>*required*|A short description of the success of the repair process. A complete repair is one which passes all of the tests in the test suite. A partial repair is one that passes at least one previously failing test and does not introduce any new test failures. If the outcome of the test suite for the best patch found during the search does cause any previously failing tests to pass, then no repair has been found.|enum (complete-repair, partial-repair, no-repair)|
|**pareto-set**  <br>*required*|A list containing details of all adaptations within the pareto set.|< [CandidateAdaptation](#candidateadaptation) > array|
|**running-time**  <br>*required*|The number of minutes taken to complete the repair process.  <br>**Minimum value** : `0`|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**204**|the TH acknowledges the done message|No Content|
|**400**|the TH has itself encountered an error processing the done message|No Content|


<a name="error-post"></a>
### POST /error

#### Description
Used to indicate that an error has occurred during the preparation
or evaluation of a test scenario, or during the start-up of the
system under test.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Error](#error)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**204**|the TH acknowledges the error|No Content|
|**400**|the TH has itself encountered an error processing the error|No Content|


#### Consumes

* `application/json`


<a name="ready-post"></a>
### POST /ready

#### Description
Used to indicate that the SUT is ready and that testing may begin.


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the TH acknowledges the ready message|[Response 200](#ready-post-response-200)|
|**400**|the TH has itself encountered an error processing the ready message|No Content|

<a name="ready-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**bugzoo-server-urls**  <br>*optional*|A list of the base URLs for all BugZoo servers that have been made available for the purpose of evaluating candidate patches.|< string > array|


#### Produces

* `application/json`


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
|**204**|the TH acknowledges the status|No Content|
|**400**|the TH has itself encontered an error processing the status|No Content|


#### Consumes

* `application/json`




<a name="definitions"></a>
## Definitions

<a name="candidateadaptation"></a>
### CandidateAdaptation

|Name|Description|Schema|
|---|---|---|
|**compilation-outcome**  <br>*required*|A description of the outcome of attempting to compile this adaptation.|[CompilationOutcome](#compilationoutcome)|
|**diff**  <br>*required*|A description of the change to the code, given in the form of a diff.|string|
|**test-outcomes**  <br>*required*|A summary of the outcomes for each of the test cases that this adaptation was evaluated against.|< [TestOutcome](#testoutcome) > array|


<a name="compilationoutcome"></a>
### CompilationOutcome

|Name|Description|Schema|
|---|---|---|
|**successful**  <br>*required*|A flag indicating whether the compilation of this adaptation was successful or not.|boolean|
|**time-taken**  <br>*required*|The number of seconds taken to compile this adaptation.  <br>**Minimum value** : `0`|number (float)|


<a name="error"></a>
### Error

|Name|Schema|
|---|---|
|**error**  <br>*required*|[error](#error-error)|

<a name="error-error"></a>
**error**

|Name|Description|Schema|
|---|---|---|
|**kind**  <br>*required*|The kind of error that occurred.  <br>**Example** : `"NeutralPerturbation"`|enum (NeutralPerturbation, FailedToComputeCoverage, NotReadyToPerturb, NotReadyToAdapt, FileNotFound, LineNotFound, OperatorNotFound, NoSearchLimits, UnexpectedError)|
|**message**  <br>*required*|Human-readable information about the error, if any can be provided.  <br>**Example** : `"invalid perturbation: no test failures."`|string|


<a name="testoutcome"></a>
### TestOutcome

|Name|Description|Schema|
|---|---|---|
|**passed**  <br>*required*|Indicates whether or not the test passed.|boolean|
|**test-id**  <br>*required*|A unique identifier for the test to which this outcome belongs.|string|
|**time-taken**  <br>*required*|The number of seconds taken to complete the test.  <br>**Minimum value** : `0`|number (float)|





