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
a linear model over the inputs (including interactions). Variations of this
power model will be provided as test inputs.

## Test Parameters

Secret power model plus all of the parameters relevant to CP1 in phase 1
(e.g. location, target location, battery level).

## Test Procedure

See overview above. In particular, this challenge problem will require a
training phase, T, where the model specified by Lincoln Labs is
learned. This will require a yet-to-be specified number of runs.

## Interface to the Test Harness (API)

Note, this API is notional at this stage. In essence, this is an evolution
of CP1 from Phase 1, with the expansion of being able to set the battery
model and place and remove obstacles multiple times during the test. The
power model, start location, and set of tasks will be specified in the test
configuration file.

```javascript
// Note, we may add additional data to the arguments as they are needed
// for LL evaluation
// Note, change from Phase I - no deadline, replaced with taskOn, which is the task currently being done
// Revised API
http://brass-th/ready
  Method: POST
  Request: No parameters
  Response: { "map_to_use" : String, "start_loc" : String, "target_loc" : String, "use_adaptation" : Boolean, "discharge_function": String, "options":array, "option_bounds":math.matrix(), "budget": Integer}
  
// Indicates that there is an error in system start or the learning process
// The TH will terminate the test if it gets this message
http://brass-th/error
  Method: POST
  Request: 
    {"ERROR" : TEST_DATA_FILE_ERROR | TEST_DATA_FORMAT_ERROR | DAS_LOG_FILE_ERROR | DAS_OTHER_ERROR | PARSING_ERROR | LEARNING_ERROR,
     "MESSAGE" : String}
  Response: No response

// Indicates to the TH important states in the SUT and DAS. Posted periodically as interesting events occur.
http://brass-th/status
   Method: POST
   Request:
     {"STATUS" : BOOTING | BOOTED | ONLINE | OFFLINE | PERTURBATION_DETECTED | MISSION_SUSPENDED | MISSION_RESUMED | MISSION_HALTED | MISSION_ABORTED | ADAPTATION_INITIATED | ADAPTATION_COMPLETED | ADAPTATION_STOPPED | TEST_ERROR | LEARNING_STARTED| LEARNING_DONE,
      "MESSAGE" : String,
      “sim_time" : Integer
     }
   Response: No response
   
// provides the data to TH
http://brass-th/action/done
   Request: 
     {"x" : Float, "y" : Float, "w" : Float, "v" : Float, 
      "charge" : batteryLevel,
      "message" : String
     } 
   Response: No response

GET http://brass-ta/action/observe
TEST_ACTION:
  {"TIME" : TIME_ENCODING, "ARGUMENTS" : {}}
ACTION_RESULT:
  {"TIME" : TIME_ENCODING,
   "RESULT" : {"x" : Float, "y" : Float, "w" : Float, "v" : Float, "voltage" : batteryLevel, “taskOn”: String, "sim_time" : Integer}
  }

// API to set up the initial conditions for the experiment for power
POST http://brass-ta/action/set_battery
TEST_ACTION:
  {"TIME" : TIME_ENCODING,
   "ARGUMENTS" : {"voltage" : batteryLevel}
  }
ACTION_RESULT:
  {"TIME" : TIME_ENCODING,
   "RESULT" : {"sim_time" : Integer}
  }
```

## Intent Specification and Evaluation Metrics

The intents described on the wiki for CP1 in phase 1 still apply (Accuracy,
timing, and safety). These should be summed over the n missions completed
by the robot. In addition, we may evaluate the discovery mechanism with a
cost function based on the number of queries used. Alternatively, Lincoln
Labs can set a tuneable query budget which will be used by the DAS.
