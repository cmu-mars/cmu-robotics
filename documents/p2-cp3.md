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
* Uncertainty and its relationship to scale. There are various sources of uncertainty in this domain that could be included as probabilistic elements in planning; we are currently investigating the best one to use in the challenge problem. The candidates for uncertainty are:
    * Partial obstacle occlusion meaning that the robot might be able to adjust is clearance parameters to squeeze past an obstacle with some probability of hitting the obstacle.
    * Probabilistic information inherent in timing or power models.
    * Probability of obstacle permanence (i.e., will an obstacle that is placed in the path stay there or be removed soon?).
    * Uncertainty in localization from sensing.
* Technologies for adapting software configurations and algorithms on-line.

## Test Data

Lincoln Labs will be able to choose from a set of predefined maps that
explore different aspects of scale and uncertainty. The selection will be
part of the configuration data for the test. This will be done through a
mnemonic in the configuration data.

## Test Parameters

> TODO: I erased what was here because it was redundant with what's
> returned by `/ready` in the description of the TH API, but I'm not sure
> what ought to be in its place.

## Test Procedure
> This automaton is obsolete according to the new API
The test procedure will be the same as for P1CP1, except that Lincoln labs will be able to perturb multiple times for each perturbation (e.g., place/remove obstacle, set battery, fail/reinstate kinect).
![runtime-automation](img/cp3-runtime-automation.png "Run-time Automation")

> Jeff: This is based on the diagram from last time. We're hoping to find a better tool to specify the state machine with. The intent is that you should be able to place (and optionally remove) obstacles simultaneously (i.e., place more than one obstacle at a time). But, this doesn't make sense for the charge and the kinect activation. Not sure how to represent this in this notation.

## Interface to the Test Harness (API)

### REST Interface to the TH

The Swagger file describing this interface is
[swagger-yaml/cp3-th.yaml](swagger-yaml/cp3-th.yaml) which should be
considered the canonical definition of the
API. [swagger-yaml/cp3-th.md](swagger-yaml/cp3-th.md) is produced
automatically from the Swagger definition for convenience.

### REST Interface to the TA

The Swagger file describing this interface is
[swagger-yaml/cp3-ta.yaml](swagger-yaml/cp3-ta.yaml) which should be
considered the canonical definition of the
API. [swagger-yaml/cp3-ta.md](swagger-yaml/cp3-ta.md) is produced
automatically from the Swagger definition for convenience.


## Intent Specification and Evaluation Metrics

### Intent Element 1: Accuracy
**Informal Description**: Robot comes to the target location.

**Formal Description**: The intent here is that if we get close to the goal (within 30cm from the center of the robot), then we get 1. Otherwise we get a linearly decreasing score the further away we are. So, if the turtlebot finishes in the green area in the figure below we get a score of 1; in the blue area we get 0 > score > 1; outside the blue circle we get 0.

**Test/Capture Method**: The position of the robot will be read from the simulator. This will be returned in test-ta/action/observed

**Verdict Expression**:

| Constant | likely value | meaning |
|----------|--------------|---------|
| BUFFER   | 50cm | the radius of the robot, plus some buffer that robotic algorithms determine as close enough |
| MAX_DISTANCE | 3m |  The maximum distance from the buffer zone that can be considered near the target. |

```
function distance(loc1, loc2) = sqrt((loc1.x - loc2.x)^2 + (loc1.y - loc2.y)^2))
```

| Condition                                                        | Score                                             |
|------------------------------------------------------------------|---------------------------------------------------|
| eventually(distance(location,target) < BUFFER)	                | 1                                                 |
| eventually(BUFFER < distance(location, target) < MAX_DISTANCE)	 | 1-(distance(location,target)-BUFFER)/(MAX_DISTANCE+BUFFER) |
| else                                                             | 0                                                 |

**Challenge evaluation for degraded intents:**

C = the challenge (with adaptation on)
B = base (with no adaptation)

DEG_C = the score (0..1) degraded of C
DEG_B = the score (0..1) degraded of B

| C ->,<br/> B \/  | PASS              | DEGRADED                             | FAIL |
|-------------|-------------------|--------------------------------------|------|
| PASS        | INCONCLUSIVE      | FAIL                                 | FAIL |
| DEGRADED    | PASS              | PASS if DEG_C > DEG_B<br/>INCONCLUSIVE if DEG_C == DEG_B<br/>FAIL otherwise | FAIL |
| FAIL        | PASS | PASS | INCONCLUSIVE |

###Intent Element 2: Timing
**Informal Description**: Robot reaches target location by a deadline

**Formal Description**:

Scoring requirements:

1. Account for some inaccuracy (can’t be right on the deadline). Call this BUFFER
2. Being early is better than being late, but don’t want to encourage inaccurate over predictions. We can be early by 2*BUFFER or late by BUFFER
3. Want to penalize too many deadline predictions (so we don’t predict just before getting there). If there is no adaptation, then we can make one prediction. Let’s allow ourselves one more prediction per adaptation that we do. We will be penalized score*(1-over_predictions)^(3/2)/15. I.e., we get penalized more severely the more we over predict. A new prediction causes the robot to send a notification to the participant.

![Timing diagram](img/cp3-timing-intent.png "Timing Intent Intent")

The above diagram gives the intent of the base score. If inDeadlineWindow, then we get 1. If too early, then we get less score, but at a slower rate (½) than if we are too late.
We’re allowed one prediction at the beginning of the test. So if there is one adaptation, then we expect two predictions.


**Test/Capture Method**: The running time of the test will be calculated starting when the test begins to when the mission is complete. The predicted deadline will be sent in the observations.

**Result Expression**: {(location, target, deadline, arrival_time, number_of_predictions, number_of_adaptations)}

**Verdict Expressions**:

| Constant | likely value | meaning |
|----------|--------------|---------|
| BUFFER   | 10 seconds   | TODO    |
| PENALTY  | 120 seconds  | TODO    |

```
function close_enough (loc1, loc2) = distance (loc1, loc2) <= MAX_DISTANCE


// can be one minute late or two minutes early
function inDeadlineWindow(deadline, arrival) = arrival <= deadline + BUFFER and arrival > deadline-2*BUFFER

// 6 minutes early is ok, but more than two minutes gives us a degraded score
function tooEarly(deadline, arrival) = arrival <= deadline - 2*BUFFER

// 3 minutes late is ok, but later than one minutes gives us a degraded score
function tooLate(deadline, arrival) = arrival > deadline + BUFFER

function prediction_penalty() = 1-(number_of_predictions - 1 - number_of_adaptations)^(3/2)/15
```

| Condition                                                      | Score                                                |
|---|---|
| eventually(close(location,target)  and inDeadlineWindow(deadline, arrival)) | 	1*prediction_penalty () |
| eventually(close(location,target)  and  tooEarly (deadline, arrival)) | prediction_penalty() * (arrival -  (deadline - 2 * (PENALTY+BUFFER)) /(2*PENALTY)) |
| eventually(close(location,target)  and tooLate(deadline, arrival)) | prediction_penalty() * ((arrival -  (deadline + PENALTY+BUFFER))  / (-PENALTY)) |
| else | 0 |


### Intent Element 3: Safety

(If safety is in play for the uncertainty requirement, we will have an intent for this; otherwise, it will not be a factor)
