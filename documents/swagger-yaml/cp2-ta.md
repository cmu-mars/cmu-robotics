# cmu mars brass ta: phase 2, cp2


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
|**Body**|**Parameters**  <br>*optional*|[Parameters](#adapt-post-parameters)|

<a name="adapt-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**attempt-limit**  <br>*optional*|An (optional) limit on the number of adaptations that may be attempted.  <br>**Minimum value** : `1`|integer|
|**hint**  <br>*optional*|We could allow hints to be provided to this method? e.g., the shape(s) or location(s) of the fix(es). TODO|string|
|**time-limit**  <br>*optional*|An (optional) time limit for the adaptation process, specified in minutes.  <br>**Minimum value** : `1`|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|successfully triggered code adaptation|No Content|
|**400**|encountered an error while triggering code adaptation|[Response 400](#adapt-post-response-400)|

<a name="adapt-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human-readable information about the error, if any can be provided|string|


<a name="perturb-post"></a>
### POST /perturb

#### Description
Applies a set of perturbations, given as a list of JSON objects, to the SUT. This endpoint should be used to prepare a test scenario for evaluation.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#perturb-post-parameters)|

<a name="perturb-post-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**perturbations**  <br>*required*|a non-empty list of perturbations to apply to the codebase|< [Perturbation](#perturbation) > array|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|applied the perturbations to the the system successfully|No Content|
|**400**|encountered an error while applying the perturbations to the system|[Response 400](#perturb-post-response-400)|

<a name="perturb-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*optional*|human-readable information about the error, if any can be provided|string|


<a name="perturbations-get"></a>
### GET /perturbations

#### Description
Returns a list of possible perturbations of an (optionally) specified shape and complexity that can be performed at a given (set of) location(s) in the program.  This endpoint should be used to select a suitable (set of) perturbation(s) for a test scenario.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**Parameters**  <br>*optional*|[Parameters](#perturbations-get-parameters)|

<a name="perturbations-get-parameters"></a>
**Parameters**

|Name|Description|Schema|
|---|---|---|
|**file**  <br>*required*|The file at which the perturbation should be injected.|string|
|**line**  <br>*optional*|The number of the line at which the perturbation should be injected. N.b. if this parameter is used, then the file parameter must also be specified.|integer|
|**shape**  <br>*required*|The shape of the fault (e.g., incorrect conditional, missing control flow).|string|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|successfully computed the list of possible perturbations|[Response 200](#perturbations-get-response-200)|
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
|**message**  <br>*optional*|human-readable information about the error, if any can be provided|string|


<a name="status-get"></a>
### GET /status

#### Description
Returns the current status of the SUT


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|successfully computed the status of the SUT|No Content|
|**400**|encounted an error while computing the status of the SUT|No Content|




<a name="definitions"></a>
## Definitions

<a name="deletestatementperturbation"></a>
### DeleteStatementPerturbation
*Polymorphism* : Inheritance  
*Discriminator* : kind


|Name|Description|Schema|
|---|---|---|
|**kind**  <br>*required*|Used to discriminate between different kinds of perturbation.|string|
|**locationRange**  <br>*optional*|The range of code that is deleted by the perturbation.|[SourceRange](#sourcerange)|


<a name="insertstatementperturbation"></a>
### InsertStatementPerturbation
*Polymorphism* : Inheritance  
*Discriminator* : kind


|Name|Description|Schema|
|---|---|---|
|**kind**  <br>*required*|Used to discriminate between different kinds of perturbation.|string|
|**location**  <br>*optional*|The location immediately before the location at which the statement will be inserted.|[SourceRange](#sourcerange)|
|**statement**  <br>*optional*|The source code for the inserted statement.|string|


<a name="perturbation"></a>
### Perturbation

|Name|Description|Schema|
|---|---|---|
|**kind**  <br>*required*|Used to discriminate between different kinds of perturbation.|string|


<a name="replacestatementperturbation"></a>
### ReplaceStatementPerturbation
*Polymorphism* : Inheritance  
*Discriminator* : kind


|Name|Description|Schema|
|---|---|---|
|**kind**  <br>*required*|Used to discriminate between different kinds of perturbation.|string|
|**locationRange**  <br>*optional*|The range of code that is replaced by the perturbation.|[SourceRange](#sourcerange)|
|**statement**  <br>*optional*|The source code for the replacement statement.|string|


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





