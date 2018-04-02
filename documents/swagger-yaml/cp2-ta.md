# CMU MARS BRASS TA: Phase II, CP2


<a name="overview"></a>
## Overview

### Version information
*Version* : 0.1


### URI scheme
*Host* : brass-ta  
*Schemes* : HTTP




<a name="paths"></a>
## Paths

<a name="adapt-post"></a>
### POST /adapt

#### Description
Used to trigger the code adaptation process.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#adapt-post-parameters)|

<a name="adapt-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**attempt-limit**  <br>*optional*|An (optional) limit on the number of adaptations that may be attempted.  <br>**Minimum value** : `1`|integer|
|**time-limit**  <br>*optional*|An (optional) time limit for the adaptation process, specified in minutes.  <br>**Minimum value** : `1`|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|Successfully triggered code adaptation|No Content|
|**400**|Encountered an error while triggering code adaptation|[Response 400](#adapt-post-response-400)|

<a name="adapt-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|Human-readable information about the error, if any can be provided|string|


#### Consumes

* `application/json`


<a name="lines-get"></a>
### GET /lines

#### Description
Returns a list of all the source lines at which perturbations may be injected.


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|Successfully computed and returned a list of source lines within the project.|< [SourceLine](#sourceline) > array|
|**400**|Failed to produce a list of source lines within the project.|No Content|


<a name="observe-get"></a>
### GET /observe

#### Description
Returns the current status of the SUT.


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|Successfully observed the state of the SUT.|[Response 200](#observe-get-response-200)|
|**400**|Encounted an error while observing the SUT.|No Content|

<a name="observe-get-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**pareto-set**  <br>*optional*|A list containing details of the sub-set of adaptations that have been encountered that belong to the pareto set (i.e., the set of non-dominated adaptations).|< [CandidateAdaptation](#candidateadaptation) > array|
|**resource-consumption**  <br>*optional*|A description of the resources that have been consumed in the process of searching for an adaptation.|[resource-consumption](#observe-get-resource-consumption)|
|**stage**  <br>*required*|A concise description of the current state of the system.|enum (awaiting-perturbation, injecting-perturbation, localising-perturbation, searching-for-adaptation, finished-adapting)|

<a name="observe-get-resource-consumption"></a>
**resource-consumption**

|Name|Description|Schema|
|---|---|---|
|**num-attempts**  <br>*required*|Number of attempted adaptations.  <br>**Minimum value** : `0`|integer|
|**time-spent**  <br>*optional*|Wall-clock time spent searching for an adaptation.  <br>**Minimum value** : `0`|number (float)|


<a name="perturb-post"></a>
### POST /perturb

#### Description
Applies a set of perturbations, given as a list of JSON objects, to the SUT. This endpoint should be used to prepare a test scenario for evaluation.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#perturb-post-parameters)|

<a name="perturb-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**perturbations**  <br>*required*|A non-empty list of perturbations to apply to the codebase|< [Perturbation](#perturbation) > array|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|Applied the perturbations to the the system successfully|No Content|
|**400**|Encountered an error while applying the perturbations to the system|[Response 400](#perturb-post-response-400)|

<a name="perturb-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|Human-readable information about the error, if any can be provided|string|


<a name="perturbations-get"></a>
### GET /perturbations

#### Description
Returns a list of possible perturbations of an (optionally) specified shape and complexity that can be performed at a given line in the program. This endpoint should be used to select a suitable (set of) perturbation(s) for a test scenario.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*required*|[Parameters](#perturbations-get-parameters)|

<a name="perturbations-get-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**file**  <br>*required*|The file at which the perturbation should be injected.|string|
|**line**  <br>*optional*|The number of the line at which the perturbation should be injected.|integer|
|**shape**  <br>*required*||[PerturbationKind](#perturbationkind)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|Successfully computed the list of possible perturbations|[Response 200](#perturbations-get-response-200)|
|**400**|Encountered an error while computing the list of possible perturbations.|[Response 400](#perturbations-get-response-400)|

<a name="perturbations-get-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**perturbations**  <br>*optional*|A list of perturbations that satisfy the query parameters provided by the request.|< [Perturbation](#perturbation) > array|

<a name="perturbations-get-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|Human-readable information about the error, if any can be provided|string|




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


<a name="perturbation"></a>
### Perturbation

|Name|Description|Schema|
|---|---|---|
|**at**  <br>*required*|The range of code that is deleted or replaced by the perturbation.|[SourceRange](#sourcerange)|
|**kind**  <br>*required*||[PerturbationKind](#perturbationkind)|
|**replacement**  <br>*optional*|The body of the source code that should replaced the source code given by the location range associated with this perturbation.|string|


<a name="perturbationkind"></a>
### PerturbationKind
A description of the kind of the perturbation.

*Type* : enum (DeleteVoidFunctionCall, FlipArithmeticOperator, FlipBooleanOperator, FlipRelationalOperator, UndoTransformation, DeleteConditionalControlFlow, FlipSignedness)


<a name="sourceline"></a>
### SourceLine

|Name|Description|Schema|
|---|---|---|
|**file**  <br>*required*|The file to which this line belongs.|string|
|**number**  <br>*required*|The one-indexed number of this line in the file.  <br>**Minimum value** : `1`|integer|


<a name="sourcelocation"></a>
### SourceLocation

|Name|Description|Schema|
|---|---|---|
|**file**  <br>*required*|The file at which this source location resides.|string|
|**offset**  <br>*required*|The character offset between the start of the file and this location.  <br>**Minimum value** : `0`|integer|


<a name="sourcerange"></a>
### SourceRange

|Name|Description|Schema|
|---|---|---|
|**start**  <br>*required*|The location that marks the start of this source range.|[SourceLocation](#sourcelocation)|
|**stop**  <br>*required*|The location that marks the end of this source range.|[SourceLocation](#sourcelocation)|


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





