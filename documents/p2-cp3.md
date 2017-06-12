# CMU MARS (Alrdich), CP3: Robot obstacle avoidance, timeliness, power, and scale

## Overview

This challenge problem is an evolution of Phase I Challenge Problem 1 that will adapt the TurtleBot's use of sensors, components, 
and mission to current energy levels to preserve mission intent and intents regarding internal behavior despite low power and *changing 
hardware conditions*.

The robot is used for a variety of tasks with different mission parameters and different mission length. In its current form the 
TurtleBot implementation does not explicitly consider energy consumption and battery life, assuming the TurtleBot can drive back 
to its home station when the battery runs low. Various ecosystem and mission changes, from obstacles on the course, to delays in 
the mission, to partial sensor failure, can interfere with those objectives.

The DAS will use both offline and online techniques to prepare possible adaptations to ecosystem changes before the mission starts. 
It will perform adaptations online during the mission.

To successfully prepare and evaluate changes, the simulator requires a sensor on the system's current energy consumption, 
e.g., whole-system energy as measured by a power meter external to the computer running the simulation or part of the computer’s 
power supply, and as approximate power consumption of the individual sensors and actuators.

The key challenges being addressed in this challenge problem are:

* Scalabilility of online adaptation planning to realistic environments and concerns
* (? Uncertainty and its relationship to safety and scale. This might be done with partial obstacle occlusion meaning that the robot might be able to adjust is clearance paramters to squeeze past an obstacle with some probability of hitting the obstacle. This introduces probabilistic elements into the plan.)
* Technologies for adapting software configurations and algorithms on-line

## Test Data

(? This challenge problem will require a set of maps that LL can select from. The "parameters" of these maps are TBD. Possible parameters are map size, connectivity [e.g., how many alternate paths], # charging stations and waypoints])

## Test Parameters

* Start and target location
* Initial battery level
* Hardware failure (e.g., (only?) kinect failure)
* (? Map)


## Test Procedure

The test procedure will be the same as for P1CP1, except that Lincoln labs will be able to perturb multiple times for each perturbation (e.g., place/remove obstacle, set battery, fail/reinstate kinect)

## Interface to the Test Harness (API)

```json
// Interface(s) from DAS/CP to test harness
// This POST message provides the status of the DAS and the system
// NOTE: Changed to use new interface
// NOTE: This is provided by the test harness -- DAS calls it
http://brass-th/ready
  {"TIME" : TIME_ENCODING}
http://brass-th/error
  {"TIME" : TIME_ENCODING, 
   "ERROR" : TEST_DATA_FILE_ERROR | TEST_DATA_FORMAT_ERROR | DAS_LOG_FILE_ERROR | DAS_OTHER_ERROR,
   "MESSAGE" : STRING_ENCODING}
http://brass-th/status
  {"TIME" : TIME_ENCODING,
   "STATUS" : PERTURBATION_DETECTED | MISSION_SUSPENDED | MISSION_RESUMED | MISSION_HALTED | MISSION_ABORTED | ADAPTATION_INITIATED | ADAPTATION_COMPLETED | ADAPTATION_STOPPED | TEST_ERROR,
   "MESSAGE" : { “msg” : STRING_ENCODING, “sim_time" : Integer }
  }
 
// a test action message that the TA may send to the TH to indicate that the mission is over before hitting time out.
// this message will be sent for one of two reasons: the bot is at the target location or the battery is about to die.
// other, more general, errors that cause the mission to end early will be reported by posting TEST_ERROR
http://brass-th/action/done
  {"TIME" : TIME_ENCODING,
   "TARGET" : STRING_ENCODING,
   "ARGUMENTS" :  { "done" : earlyTermReason }
  } 
// Interfaces from test harness to DAS/CP
// After ready is reported, the th will use this query to 
// get the initial planned path. note that waypoint names are unique.
// time gives a lower bound on the number of seconds we estimate to 
// traverse the path.
GET http://brass-ta/action/query_path
TEST_ACTION:
  {"TIME": TIME_ENCODING, "ARGUMENTS" : {}}
ACTION_RESULT:
  {"TIME": TIME_ENCODING, "RESULT" : {"path" : [String], "time" : Float}}
  
// Starts the TurtleBot navigating through the map for challenge problem 1
POST http://brass-ta/action/start
  {"TIME" : TIME_ENCODING, "ARGUMENTS" : {}}
 
// Note, we may add additional data to the arguments as they are needed
// for LL evaluation
GET http://brass-ta/action/observe
TEST_ACTION:
  {"TIME" : TIME_ENCODING, "ARGUMENTS" : {}}
ACTION_RESULT:
  {"TIME" : TIME_ENCODING, 
   "RESULT" : {"x" : Float, "y" : Float, "w" : Float, "v" : Float, "voltage" : batteryLevel, "deadline" : Integer, "sim_time" : Integer}
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
 
// tries to place an obstacle at the argument (x,y). if possible, returns a unique name for that obstacle and the (x,y)
// of the top left and bottom right corner of the unsafe region. 
POST http://brass-ta/action/place_obstacle
TEST_ACTION:
  {"TIME" : TIME_ENCODING,
   "ARGUMENTS" : {"x" : Float, "y" : Float}}
ACTION_RESULT:
  {"TIME" : TIME_ENCODING,
   "RESULT" : {"obstacleid" : STRING_ENCODING, topleft_x : Float, topleft_y : Float, botright_x : Float, botright_y : Float, "sim_time" : Integer }}
 
// removes an obstacle when given its name as returns from /action/place_obstacle
POST http://brass-ta/action/remove_obstacle
TEST_ACTION:
  {"TIME" : TIME_ENCODING,
   "ARGUMENTS" : {"obstacleid" : STRING_ENCODING}}
ACTION_RESULT:
  {"TIME" : TIME_ENCODING,
   "RESULT" : {"sim_time" : Integer}}
 
POST http://brass-ta/action/perturb_sensor
TEST_ACTION:
  {"TIME" : TIME_ENCODING,
   "ARGUMENTS" : {"status" : "on" | "off"}
  }
ACTION_RESULT:
  {"TIME" : TIME_ENCODING,
   "RESULT" : {"sim_time" : Integer}
  }
```

## Intent Specification and Evaluation Metrics

* Accuracy, as per P1CP1
* Timeliness, as per P1CP1
* (not safety?)
