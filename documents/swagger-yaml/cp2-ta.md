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
Triggers the code adaptation process.


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
|**204**|OK.|No Content|
|**400**|An error prevented code adaptation from being triggered.|[Error](#error)|
|**409**|System has not been (successfully) perturbed or adaptation has already been triggered.|[Error](#error)|


#### Consumes

* `application/json`


<a name="files-get"></a>
### GET /files

#### Description
Returns a list of all the source files that may be subject to perturbation.


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|OK.|< string > array|


<a name="lines-get"></a>
### GET /lines

#### Description
Returns a list of all the source lines at which perturbations may be injected.


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|OK.|< [SourceLine](#sourceline) > array|


<a name="observe-get"></a>
### GET /observe

#### Description
Returns the current status of the SUT.


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|OK.|[Response 200](#observe-get-response-200)|

<a name="observe-get-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**pareto-set**  <br>*optional*|A list containing details of the sub-set of adaptations that have been encountered that belong to the pareto set (i.e., the set of non-dominated adaptations).|< [CandidateAdaptation](#candidateadaptation) > array|
|**resource-consumption**  <br>*optional*|A description of the resources that have been consumed in the process of searching for an adaptation.|[resource-consumption](#observe-get-resource-consumption)|
|**stage**  <br>*required*|A concise description of the current state of the system.|enum (READY_TO_PERTURB, PERTURBING, READY_TO_ADAPT, SEARCHING, FINISHED)|

<a name="observe-get-resource-consumption"></a>
**resource-consumption**

|Name|Description|Schema|
|---|---|---|
|**num-attempts**  <br>*required*|Number of attempted adaptations.  <br>**Minimum value** : `0`|integer|
|**time-spent**  <br>*optional*|Wall-clock time spent searching for an adaptation.  <br>**Minimum value** : `0`|number (float)|


<a name="perturb-post"></a>
### POST /perturb

#### Description
Applies a given perturbation to the SUT. The resulting perturbed system will become Baseline B, provided that the system builds and fails at least one test.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**perturb-params**  <br>*required*|[Perturbation](#perturbation)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**204**|OK.|No Content|
|**400**|Failed to perturb the SUT.|[Error](#error)|
|**409**|The SUT has already been perturbed.|[Error](#error)|


<a name="perturbations-get"></a>
### GET /perturbations

#### Description
Returns a list of possible perturbations of an (optionally) specified shape and complexity that can be performed at a given line in the program. This endpoint should be used to select a suitable (set of) perturbation(s) for a test scenario.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**perturbation-params**  <br>*required*|[perturbation-params](#perturbations-get-perturbation-params)|

<a name="perturbations-get-perturbation-params"></a>
**perturbation-params**

|Name|Description|Schema|
|---|---|---|
|**file**  <br>*required*|The file at which the perturbation should be injected.|string|
|**line**  <br>*optional*|The number of the line at which the perturbation should be injected.|integer|
|**shape**  <br>*required*||[PerturbationKind](#perturbationkind)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|OK.|[Response 200](#perturbations-get-response-200)|
|**400**|Encountered an error while computing the list of possible perturbations.|[Error](#error)|

<a name="perturbations-get-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**perturbations**  <br>*optional*|A list of perturbations that satisfy the query parameters provided by the request.|< [Perturbation](#perturbation) > array|




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


<a name="perturbation"></a>
### Perturbation

|Name|Description|Schema|
|---|---|---|
|**arguments**  <br>*required*||< string, string > map|
|**at**  <br>*required*|The range of code that is deleted or replaced by the perturbation.|[SourceRange](#sourcerange)|
|**kind**  <br>*required*||[PerturbationKind](#perturbationkind)|
|**replacement**  <br>*optional*|The body of the source code that should replaced the source code given by the location range associated with this perturbation.|string|
|**transformation-index**  <br>*required*|**Minimum value** : `0`|integer|


<a name="perturbationkind"></a>
### PerturbationKind
A description of the kind of the perturbation.

*Type* : enum (delete-void-function-call, flip-arithmetic-operator, flip-boolean-operator, flip-relational-operator, undo-transformation, delete-conditional-control-flow, flip-signedness)


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
|**column**  <br>*required*|The one-indexed column number of this location.  <br>**Minimum value** : `1`|integer|
|**file**  <br>*required*|The file at which this source location resides.|string|
|**line**  <br>*required*|The one-indexed line number of this location.  <br>**Minimum value** : `1`|integer|


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
|**passed**  <br>*required*|Indicates whether or not the test passed.|boolean|
|**test-id**  <br>*required*|A unique identifier for the test to which this outcome belongs.|string|
|**time-taken**  <br>*required*|The number of seconds taken to complete the test.  <br>**Minimum value** : `0`|number (float)|





