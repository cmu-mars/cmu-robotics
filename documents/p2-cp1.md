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

## Interface to the Test Harness (API)

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

```javascript
http://brass-th/ready
  Method: POST
  Request: No parameters
  Response: {"mode": Integer, "start_loc": String, "target_locs": Array, "num_of_waypoints": Integer,
              "discharge_function": String, "budget_discharge": Integer,
              "charge_function": String, "budget_charge": Integer}

// Indicates that there is an error in system start or the learning process
// The TH will terminate the test if it gets this message
// Specific to CP1 are PARSING_ERROR and LEARNING_ERROR that may happen when something went wrong during
// the model parsing and model evaluation, or during the learning process, note that we have two separate
// packages for model parsing and evaluation as well as model learning. Model learning package use the
// parser to evaluate expression during learning, so the error might happen during the model evaluation
// (evaluating the expression of power model), or learning process.
http://brass-th/error
  Method: POST
  Request:
    {"ERROR": TEST_DATA_FILE_ERROR | TEST_DATA_FORMAT_ERROR | DAS_LOG_FILE_ERROR | DAS_OTHER_ERROR | PARSING_ERROR | LEARNING_ERROR,
     "MESSAGE": String}
  Response: No response

// Indicates to the TH important states in the SUT and DAS. Posted periodically as interesting events occur.

http://brass-th/status
   Method: POST
   Request:
     {"STATUS": BOOTING | BOOTED | ONLINE | OFFLINE | PERTURBATION_DETECTED | MISSION_SUSPENDED
                | MISSION_RESUMED | MISSION_HALTED | MISSION_ABORTED | ADAPTATION_INITIATED
                | ADAPTATION_COMPLETED | ADAPTATION_STOPPED | TEST_ERROR | LEARNING_STARTED | LEARNING_DONE,
      "x" : Float, "y" : Float, "w" : Float, "v" : Float,
      "charge" : batteryLevel, "predicted_arrival" : Integer,
      "sim_time" : Integer
     }
   Response: No response

// provides the data to TH
http://brass-th/action/done
   Method: POST
   Request:
     {"x" : Float, "y" : Float, "w" : Float, "v" : Float,
      "charge" : batteryLevel, "predicted_arrival" : Integer,
      "sim_time" : Integer,
      "message" : String
     }
   Response: No response

//
// Here are the APIs related to perturbations
//

// This will inject perturbations such as setting new battery charge, placing obstacles, removing obstacles.
// errors to be redirected to TH /error
// if the perturbation injected successfully, we should observe PERTURBATION_DETECTED in the /status

// tries to place an obstacle at the argument (x,y). if possible, returns a unique name for that obstacle and the (x,y)
// of the top left and bottom right corner of the unsafe region.
http://brass-ta/perturb/place_obstacle
   Method: POST
   Request:
     {"x" : Float, "y" : Float,
       "type" : String}
   Response:
     {"obstacleid" : STRING_ENCODING,
      "topleft_x" : Float, "topleft_y" : Float, "botright_x" : Float, "botright_y" : Float,
      "sim_time" : Integer }

http://brass-ta/perturb/remove_obstacle
   Methods: POST
   Request: {"obstacleid" : STRING_ENCODING}
   Result: {"sim_time" : Integer}

http://brass-ta/perturb/battery
   Method: POST
   Request: {"charge" : batteryLevel}
   Response: {"sim_time" : Integer}

```

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
