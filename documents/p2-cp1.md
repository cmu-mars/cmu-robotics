# CMU MARS (Aldrich), CP1: Integrated power model discovery and adaptation

## Overview

The goal of this challenge problem is to discover the power model of the
mobile robotics platform and use the discovered model to adapt for optimal
mission performance. The intent of this challenge problem is to simulate
robots with different hardware, algorithms, and workload and therefore
different power usage characteristics, in which we intentionally simulate
also severe changes that might be caused by unanticipated and yet-unknown
future environment changes. This challenge problem tests whether we can
adapt our robotic platform successfully to such situations.

This challenge problem is an extension of the MARS team's CP1 in the first
phase, "Low Power Adaptation." We expect the primary evaluation functions
and intent elements will remain the same.

Lincoln Labs will select a secret power model which is a function with a
large number of inputs (e.g. `20`) and outputs energy consumption over
time. That is, Lincoln Labs can provide drastic differences for energy
consumption of hardware and software components that may not reflect
current but possibly distant-future hardware and software. There will be a
query mechanism whereby the MARS DAS can provide a particular set of inputs
and receive back the related output. In addition, the power model is used
during the evaluation to compute the battery level during simulation.

The challenge problem will proceed in two phases. First, the DAS will use
the query mechanism to query the power consumption for certain inputs
(simulating the idea of running experiments in practice to measure power
consumption in specific configurations). The DAS will query only a small
number of inputs up to a query budget is reached. The DAS will use the
information gleaned to learn an approximation of the secret power model.

In the second phase, the robot will be asked to complete `n` missions
within a map without running out of energy. There are charging stations on
the map. The learned approximation of the power model is used by the
adaptation logic, but the actual battery levels during the evaluation are
computed with the secret power model provided by Lincoln Labs. In the
baseline scenario, the MARS team will use either a hard-coded energy model
or no energy model at all. The robot would therefore not make effective
planning decisions about whether and when to recharge. In the adaptive
cases, the MARS team will use the energy model learned in
phase 1. Differences should be observable in terms of mission failures
(e.g., running out of energy) and completion time (e.g., recharging more
effectively with fewer disruptions to the mission).


## Test Data

We will provide a baseline power model that describes the power consumption
of the system depending on a number of configuration options. The model is
a linear model over the inputs (including interactions).

> I presume we do not want to let LL to change the model? Variations of
> this power model will be provided as test inputs.

Possible perturbations that we consider in this challenge problem:
* Placement of 1 obstacle once or multiple times
* Placement of multiple obstacle once or multiple times
* Placement and removal of obstacle
* Set charge (to a higher or lower degree)
* Sequence of target endpoints (multiple missions)

## Test Parameters

Secret power models (for charge and discharge) plus all of the parameters
relevant to CP1 in phase 1 (e.g. budget start location, target location,
battery level). We need to detail specific parameters for each of the
perturbations that we decide (e.g., number of obstacles, frequency based
on which obstacles are put, number of target points, set to a higher/lower
charge, etc).

## Test Procedure

See overview above. In particular, this challenge problem will require a
training phase, Tr, where the model specified by Lincoln Labs is
learned. This requires a budget (number of times the hidden function will
be queried) that will be given by LL. We learn the function once at the
beginning offline and then the online phase will be started.

> **TODO**: Sequence Diagram

## Notes about both REST APIs

Note, this API is notional at this stage.

* mode encodes: (pert|adaptation) & (No PM|Predefined PM|Learned PM)
* num_of_waypoints is the number of target points, i.e., sub-missions that
  needs to be completed mode is one of the following cases, cf. table below:
   1. A (no perturbation, no adaptation) and no power model so the robot do
   not have a clue to charge even when the battery goes bellow a threshold

   2. B (perturbation, no adaptation) and no power model so the robot do
   not have a clue to charge even when the battery goes bellow a threshold

   3. C (perturbation, adaptation) and a static predefined power model (we
   implicitly assume this is inaccurate) so the planner uses an inaccurate
   model for planning an adaptation

   4. C (perturbation, adaptation) and a learned model that the planner use
   for adaptation

* The discharge and charge functions are what we mean by the power models.

  These are linear combinations of polynomial features with three different
  variables and interactions between them.

  More specifically, a power model is specified by the following general definition (formula):
   $f(o_1,\cdots,o_d) = \beta_0 + \sum_{o_i \in \mathcal{O}} \phi_i (o_i)
                        + \sum_{o_{i..j} \in \mathcal{O}} \Phi_i
  (o_{i..j})$, where $\beta_0$ represents a constant term, $\phi_i (o_i)$
  represents terms containing individual options, $\Phi_i (o_{i..j})$
  represents terms containing multiple options.

  Here, the parameters of the model (variables) are speed of the robot
  ($s$), kinect components ($k$) and localization algorithms ($l$),
  therefore, $d=3$.

* For example, consider the following function as a discharge function:
  $f(s,k,l)=2+3*s+1.2*k+10*k*l$, this means that the battery of the robot
  will be discharge according to $b'=b-(2+3*s+1.2*k +10*k*l)*t$, where $t$
  is the time unit.

  In this discharge function, $s=1$ represent half speed and $s=2$
  represents full speed, $k=1$ is the least accurate kinect and less energy
  consuming battery while $k=5$ is the most energy consuming one (note we
  will implement 5 different kinects that have different resolutions, etc),
  also 5 different localization where $l=1$ is the less energy consuming
  while $l=5$ is the most energy consuming one.

  Therefore, we encode the values of each variables by $\{1 2 3 4 5\}$
  sorted by least energy consuming to the most.

  In this model, $k,l$ are interacting via the last terms in the model,
  also the coefficients of the model reflects the effect of each
  variable. Each of these variables could be a polynomial combinations of
  the three variables.

  So, LL specifies the function with three variables by determining the
  coefficients and the terms of the model.

* Specific to CP1 are PARSING_ERROR and LEARNING_ERROR that may happen when
  something went wrong during the model parsing and model evaluation, or
  during the learning process, note that we have two separate packages for
  model parsing and evaluation as well as model learning. Model learning
  package use the parser to evaluate expression during learning, so the
  error might happen during the model evaluation (evaluating the expression
  of power model), or learning process.


## REST Interface to the TH

<a name="action-done-post"></a>
### POST /action/done

#### Description
used by the TA to indicate to the TH that the turtlebot has reached the goal and that the mission has been completed. note that incomplete missions will result in an error and not use this end point.


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**error info**  <br>*optional*|[error info](#action-done-post-error-info)|

<a name="action-done-post-error-info"></a>
**error info**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*required*|final charge measure of the turtlebot TODO - check ranges  <br>**Minimum value** : `16000`  <br>**Maximum value** : `32000`|integer|
|**message**  <br>*required*|TODO human readable something or other|string|
|**predicted_arrival**  <br>*required*|final best prediction of arrival time, in simulation time  <br>**Minimum value** : `0`|integer|
|**sim_time**  <br>*required*|the final internal simulation time  <br>**Minimum value** : `0`|integer|
|**v**  <br>*required*|final ?TODO? of the turtlebot|number (float)|
|**w**  <br>*required*|final ?TODO? of the turtlebot|number (float)|
|**x**  <br>*required*|final x-coordinate of the turtlebot|number (float)|
|**y**  <br>*required*|final y-coordinate of the turtlebot|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH acknowledges the completion of the mission|No Content|
|**400**|TH encountered an error at the completion of the mission|No Content|


<a name="error-post"></a>
### POST /error

#### Description
used by the TA to indicate to the TH that an error has occurred in the start up or learning process. the TH will terminate the test upon notification of an error


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**error info**  <br>*optional*|[error info](#error-post-error-info)|

<a name="error-post-error-info"></a>
**error info**

|Name|Description|Schema|
|---|---|---|
|**ERROR**  <br>*required*|one of a enumerated set of reasons that errors may arise<br>  * TEST_DATA_FILE_ERROR - TODO<br>  * TEST_DATA_FORMAT_ERROR - TODO<br>  * DAS_LOG_FILE_ERROR - TODO<br>  * DAS_OTHER_ERROR - TODO<br>  * PARSING_ERROR - TODO<br>  * LEARNING_ERROR - TODO|enum (TEST_DATA_FILE_ERROR, TEST_DATA_FORMAT_ERROR, DAS_LOG_FILE_ERROR, DAS_OTHER_ERROR, PARSING_ERROR, LEARNING_ERROR)|
|**MESSAGE**  <br>*required*|human readable text describing the error|string|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH acknowledges the error and is shutting down the test|No Content|


<a name="ready-post"></a>
### POST /ready

#### Description
indicate to the TH that the TA is ready to recieve configuration data to continue starting up the DAS


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH is ready to produce configuration data|[Response 200](#ready-post-response-200)|
|**400**|TH encountered an error producing configuration data|No Content|

<a name="ready-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**charge_budget**  <br>*optional*|the maximum number of queries against the target recharging function during learning|integer|
|**charge_function**  <br>*optional*|a description of the function dictating the recharging of the battery, which is what we will learn. TODO - this can't be a string|string (function_spec)|
|**discharge_budget**  <br>*optional*|the maximum number of queries against the target function during learning|integer|
|**discharge_function**  <br>*optional*|a description of the function dictating the discharge of the battery, which is what we will learn. TODO - this can't be a string|string (function_spec)|
|**level**  <br>*optional*|the level at which the DAS should operate for this test<br>  * pert - TODO<br>  * no_adaptation - TODO|enum (pert, no_adaptation)|
|**model**  <br>*optional*|the type of power model to use for this test<br>  * no_pm - TODO<br>  * predefined_pm - TODO<br>  * learned_pm - TODO|enum (no_pm, predefined_pm, learned_pm)|
|**start_loc**  <br>*optional*|the name of the start map waypoint|string|
|**target_locs**  <br>*optional*|the names of the waypoints to visit, in the order in which they must be visited|< string > array|


<a name="status-post"></a>
### POST /status

#### Description
used by the TA to periodically indicate its current state to the TH


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**error info**  <br>*optional*|[error info](#status-post-error-info)|

<a name="status-post-error-info"></a>
**error info**

|Name|Description|Schema|
|---|---|---|
|**STATUS**  <br>*required*|one of the possible statuses<br> * BOOTING - TODO<br> * BOOTED - TODO<br> * ONLINE - TODO<br> * OFFLINE - TODO<br> * PERTURBATION_DETECTED - TODO<br> * MISSION_SUSPENDED - TODO<br> * MISSION_RESUMED - TODO<br> * MISSION_HALTED - TODO<br> * MISSION_ABORTED - TODO<br> * ADAPTATION_INITIATED - TODO<br> * ADAPTATION_COMPLETED - TODO<br> * ADAPTATION_STOPPED - TODO<br> * TEST_ERROR - TODO<br> * LEARNING_STARTED - TODO<br> * LEARNING_DONE - TODO|enum (BOOTING, BOOTED, ONLINE, OFFLINE, PERTURBATION_DETECTED, MISSION_SUSPENDED, MISSION_RESUMED, MISSION_HALTED, MISSION_ABORTED, ADAPTATION_INITIATED, ADAPTATION_COMPLETED, ADAPTATION_STOPPED, TEST_ERROR, LEARNING_STARTED, LEARNING_DONE)|
|**charge**  <br>*required*|current turtlebot battery charge in mWh TODO - are these mins and maxes right?  <br>**Minimum value** : `16000`  <br>**Maximum value** : `32000`|integer|
|**predicted_arrival**  <br>*required*|current best prediction of arrival time, in simulation time  <br>**Minimum value** : `0`|integer|
|**sim_time**  <br>*required*|the internal simulation time at the time that the status message was sent  <br>**Minimum value** : `0`|integer|
|**v**  <br>*required*|current ? TODO ? of the turtlebot|number (float)|
|**w**  <br>*required*|current ? TODO? of the turtlebot|number (float)|
|**x**  <br>*required*|current x-coordinate of the turtlebot|number (float)|
|**y**  <br>*required*|current y-coordinate of the turtlebot|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|TH acknowledges the status message|No Content|
|**400**|TH encountered an error with the status message|No Content|

## REST Interface to the TA

<a name="perturb-battery-post"></a>
### POST /perturb/battery

#### Description
set the level of the battery in a currently running test


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**place obstacle parameters**  <br>*optional*|[place obstacle parameters](#perturb-battery-post-place-obstacle-parameters)|

<a name="perturb-battery-post-place-obstacle-parameters"></a>
**place obstacle parameters**

|Name|Description|Schema|
|---|---|---|
|**charge**  <br>*required*|the level to which the battery should be set, in mWh. TODO -- are those the right constants? i have no idea; does it matter if battery sets are monotonically decreasing  <br>**Minimum value** : `16000`  <br>**Maximum value** : `32000`|number|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the battery has been set to the requested level|[Response 200](#perturb-battery-post-response-200)|
|**400**|an error was encountered while setting the battery|[Response 400](#perturb-battery-post-response-400)|

<a name="perturb-battery-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**sim_time**  <br>*required*|the simulation time when the battery was set|integer|

<a name="perturb-battery-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**message**  <br>*required*|human readable info about what went wrong|string|


<a name="perturb-place_obstacle-post"></a>
### POST /perturb/place_obstacle

#### Description
if the test is running, place an obstacle on the map


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**place obstacle parameters**  <br>*optional*|[place obstacle parameters](#perturb-place_obstacle-post-place-obstacle-parameters)|

<a name="perturb-place_obstacle-post-place-obstacle-parameters"></a>
**place obstacle parameters**

|Name|Description|Schema|
|---|---|---|
|**kind**  <br>*required*|the name of the shape of object to be placed. TODO -- make this an enum not a string|string|
|**x**  <br>*required*|the x-coordinate of the center of the obstacle placement position|number (float)|
|**y**  <br>*required*|the y-coordinate of the center of the obstacle placement position|number (float)|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the obstacle has been placed in the running test|[Response 200](#perturb-place_obstacle-post-response-200)|
|**400**|an error was encountered while placing the obstacle.|[Response 400](#perturb-place_obstacle-post-response-400)|

<a name="perturb-place_obstacle-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**botright_x**  <br>*required*|the x-coordinate of the bottom right corner of the bounding box of the placed obstacle|number (float)|
|**botright_y**  <br>*required*|the y-coordinate of the bottom right corner of the bounding box of the placed obstacle|number (float)|
|**obstacleid**  <br>*required*|a unique identifier for this particular placed obstacle, so that it can be removed in the future|string|
|**sim_time**  <br>*required*|the simulation time when the obstacle was placed|integer|
|**topleft_x**  <br>*required*|the x-coordinate of the top left corner of the bounding box of the placed obstacle|number (float)|
|**topleft_y**  <br>*required*|the y-coordinate of the top left corner of the bounding box of the placed obstacle|number (float)|

<a name="perturb-place_obstacle-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**cause**  <br>*required*|a reason for the error condition|enum (bad_coordiantes, other_error)|
|**message**  <br>*required*|human readable info about what went wrong|string|


<a name="perturb-remove_obstacle-post"></a>
### POST /perturb/remove_obstacle

#### Description
if the test is running, remove a previously placed obstacle from the map


#### Parameters

|Type|Name|Schema|
|---|---|---|
|**Body**|**place obstacle parameters**  <br>*optional*|[place obstacle parameters](#perturb-remove_obstacle-post-place-obstacle-parameters)|

<a name="perturb-remove_obstacle-post-place-obstacle-parameters"></a>
**place obstacle parameters**

|Name|Description|Schema|
|---|---|---|
|**obstacleid**  <br>*required*|the obstacle ID given by /perturb/place_obstacle of the obstacle to be removed|string|


#### Responses

|HTTP Code|Description|Schema|
|---|---|---|
|**200**|the obstacle has been removed from the running test|[Response 200](#perturb-remove_obstacle-post-response-200)|
|**400**|an error was encountered while removing the obstacle.|[Response 400](#perturb-remove_obstacle-post-response-400)|

<a name="perturb-remove_obstacle-post-response-200"></a>
**Response 200**

|Name|Description|Schema|
|---|---|---|
|**sim_time**  <br>*required*|the simulation time when the obstacle was placed|integer|

<a name="perturb-remove_obstacle-post-response-400"></a>
**Response 400**

|Name|Description|Schema|
|---|---|---|
|**cause**  <br>*required*|a reason for the error condition|enum (bad_obstacleid, other_error)|
|**message**  <br>*required*|human readable info about what went wrong|string|


## Intent Specification and Evaluation Metrics

The intents described on the Wiki for CP1 in phase 1 still apply (Accuracy,
timing, and safety). Also, we may consider power consumptions as a metric
for evaluation. These should be summed over the `N` missions completed by
the robot. In addition, we may evaluate the discovery mechanism with a cost
function based on the number of queries used. Alternatively, Lincoln Labs
can set a tunable query budget which will be used by the DAS.

|             | A (p:✕,a:✕) | B (p:✔,a:✕) | C (p:✔,a:✔) |
|-------------|-------------|-------------|-------------|
| No PM       | `✔`         | `✔`         |             |
| Predefined  |             |             | `✔`         |
| Learned     |             |             | `✔`         |

We implicitly mean predefined is an inaccurate model.

To evaluate intent discovery, we propose that a set of test cases, each
describing a mission as well as perturbations for the robot (e.g.,
navigating a simulated corridor, placing 1 obstacle and changing the
battery level once). We should classify the test cases as `easy, medium,
difficult, very difficult, impossible`. We use metrics such as distance
from the target, power consumption, etc to evaluate the success of failure
of the mission. We measure quality as an approximate measure of how closely
the behavior of a system meets its intent. In this challenge problem we
evaluate how adaptations made by planner that uses a learned model
partially restore intent (e.g., switching to an alternative kinect, less
accurate navigation algorithm).

Each test case is described by the following:

 * Mission schema: Navigation
 * Mission parameters: A->B
 * Perturbations: Obstacles + Battery level change
 * Possible adaptations: Kinects we can swap, Algorithms we may downgrade, etc
 * Evaluation metric: Power consumed, Mission accomplish time, Distance to
   target location, Number of times we hit obstacles
