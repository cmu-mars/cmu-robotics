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

```javascript

// Revised API
// mode encodes: (pert|adaptation) & (No PM|Predefined PM|Learned PM)
http://brass-th/ready
  Method: POST
  Request: No parameters
  Response: {"mode": Integer, "start_loc": String, "target_loc": String, "discharge_function": String, "budget": Integer}

// Indicates that there is an error in system start or the learning process
// The TH will terminate the test if it gets this message
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
     {"STATUS": BOOTING | BOOTED | ONLINE | OFFLINE | PERTURBATION_DETECTED | MISSION_SUSPENDED | MISSION_RESUMED | MISSION_HALTED | MISSION_ABORTED | ADAPTATION_INITIATED | ADAPTATION_COMPLETED | ADAPTATION_STOPPED | TEST_ERROR | LEARNING_STARTED | LEARNING_DONE,
      "MESSAGE": String,
      "sim_time": Integer
     }
   Response: No response

// provides the data to TH
http://brass-th/action/done
   Method: POST
   Request:
     {"x" : Float, "y" : Float, "w" : Float, "v" : Float,
      "charge" : batteryLevel, "num_hidden_func_query": Integer, "sim_time": Integer, "num_adaptations": Integer
      "learning_status": Boolean, "num_learned_func_query": Integer,
      "message" : String
     }
   Response: No response
```

## Interface to the TA (API)

Note, this API is notional at this stage.

```javascript
//
// Here are the APIs related to perturbations and adaptation triggers, internal APIs??
//

// This will inject perturbations such as changing the discharge function, or setting new/in itializing battery charge, placing obstacles, removing obstacles, or changing kinect type or changing any other components of the system that typically affect the performance and discharge battery level differently.
// Do we need to have different discharge functions that we need to discover based on components that will be replaced at runtime? If so, every time we change this via /perturb, we need to change the hidden function and call /learn
// errors to be redirected to TH /error
// if the perturbation injected successfully, we should observe PERTURBATION_DETECTED in the /status
http://brass-ta/perturb
   Method: POST
   Request:
     {"ID": Integer,
      "parameters": Array
     }
   Response: No response

// This will trigger the learning process
// Should this method return the status whether the function has been learned? or we get this via /status?
http://brass-ta/action/learn
   Method: POST
   Request:
    {"budget": Integer
    }
   Response: No response

// This will trigger the adaptation process
http://brass-ta/adapt
   Method: POST
   Request:
    {"num_attempts": Integer, "charge" : batteryLevel
    }
   Response: No response

// Enables/disables the DAS
http://brass-ta/das
   Method: POST
   Request: {"enabled" : Boolean}
   Response: No response

// start the mission a->b
http://brass-ta/action/start
  Method: POST
  Request: No parameters
  Response: No response

// Note, we may add additional data to the arguments as they are needed
// for LL evaluation, how this should be different from th/status? i.e., when we should call each? should we merge the two?
http://brass-ta/observe
   Method: GET
   Request: No parameters
   Response:
     {"x" : Float, "y" : Float, "w" : Float, "v" : Float,
      "charge" : batteryLevel, "predicted_arrival" : Integer,
      "kinect_status" : "on" | "off",
      "sim_time" : Integer
     }

```

## Intent Specification and Evaluation Metrics

The intents described on the wiki for CP1 in phase 1 still apply (Accuracy,
timing, and safety). Also, we may consider power consumptions as a metric
for evaluation.  These should be summed over the n missions completed by
the robot. In addition, we may evaluate the discovery mechanism with a cost
function based on the number of queries used. Alternatively, Lincoln Labs
can set a tunable query budget which will be used by the DAS.

|             | A (p:✕,a:✕) | B (p:✔,a:✕) | C (p:✔,a:✔) |
|-------------|-------------|-------------|-------------|
| No PM       | ✔           | ✔           | ✔           |
| Predefined  | ✔           | ✔           | ✔           |
| Learned     |             |             | ✔           |

To evaluate intent discovery, we propose that a set of test cases, each
describing a mission as well as perturbations for the robot (e.g.,
navigating a simulated corridor, placing 1 obstacle and changing the
battery level once). We should classify the test cases as `easy, medium,
difficult, very difficult, impossible`.  We use metrics such as distance
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
