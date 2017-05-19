# Team Name
# Challenge Problem Number and Name

## Overview

Provide a short description of the challenge problem. An example of this
would be the challenge problem overview provided in the Phase 1 evaluation
reports which describe the challenge problem scenario and the key mission
goals and technology employed to support this challenge problem.

The following questions should be thought about and addressed somewhere in
this document.

* Describe the mission scenario of the platform/challenge problem.

  * Please provide more detail about the platform if it is new or different
    from what was presented in Phase 1.

* What will this challenge problem focus on demonstrating?

* What are the key technologies/enablers for this challenge problem
  (i.e. online machine learning and assurance monitors will be present in
  Phase 2 to allow the system to...).

* How would success be measured?

## Test Data

Describe any specific data that will be used or need to be generated to
test the challenge problem. An example of this from Phase 1 was CRA’s use
of weather station data as a proxy for actual underwater vehicle sensor
data.

If your challenge problem will not need any specific data sets to be
present or generated just provide a short note stating such.

## Test Parameters

Describe the different parameters (some teams call these knobs) that will
be explored in this Phase. These parameters represent the input state-space
of ecosystem changes presented to the system under test. Be as specific as
possible to the types and values for each parameter. The following table
format can be used to describe the parameters of the system under test.

TABLE GOES HERE

## Test Procedure

Describe how the Test Harness will interact with the Challenge Problem and
platform under test. It is expected that there will be more dynamic
engagement at runtime between the test harness and system under test in
Phase 2 as the designs become more complex. This may include changes to the
underlying ecosystem but also the ability of the test harness to monitor
critical aspects of the system.

Explain if there is any deviation from the testing strategy used in Phase 1
(Baseline A, Baseline B, Challenge Stage). Describe what it means to test
the challenge problem in each of those stages (i.e. Baseline A will choose
settings for knobs A and C, B will always be set to its default value –
Baseline B and the Challenge Stage use the same knob settings for A and C
chosen in Baseline A. In addition, a new setting for knob B will be
presented to Baseline B and Challenge Stage).

Baseline A – Description

Baseline B – Description

Challenge Stage – Description

Be specific and describe any testing strategies that should be
followed. For example, you may want to define a number of test cases where
a certain subset of the input state-space is explored while holding others
constant.

## Interface to the Test Harness (API)

Based on the Test Procedure described above and knowledge of the existing
Phase 1 API propose an initial cut at what the API would look like to
support the interaction as described. This should include parameters to
initialize the system and those which will change at runtime through calls
into the system under test.

The [CMU MARS team Phase 1 wiki
page](https://wikis.mit.edu/confluence/display/BRASS/CMU+MARS+Phase+1+Challenge+Problem+Announcement)
provides a good example of defining these calls (example table below).

```javascript
// Interface for test harness to observe the state of the robot
GET http://brass-ta/action/observe
TEST_ACTION:
  {"TIME" : TIME_ENCODING, "ARGUMENTS" : {}}
   ACTION_RESULT:
   {"TIME" : TIME_ENCODING,
    "RESULT" : {"x" : Float,
                "y" : Float,
                "w" : Float,
                "v" : Float,
                "voltage" : batteryLevel,
                "deadline" : Integer,
                "sim_time" : Integer
               }
   }
```

```javascript
// API to set up the initial conditions for the robot voltage level
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

An interaction diagram like the following would also be helpful in our
discussions.

![TH-to-TA interaction diagram](/img/template-diagram.png)

## Intent Specification and Evaluation Metrics

Describe if there is a new process for discovering/specifying intent of the
challenge problem in Phase 2. How will the challenge problem allow Lincoln
to measure intent preservation?

In Phase 1 teams chose different ways to express intent preservation. For
some, it was a mission goal which needed to be satisfied (e.g. the robot
made it within some distance measure to the target – comparison against a
fixed threshold). For others, it was a quantitative comparison of how well
the adaptive system performed relative to a non-adaptive baseline.

Describe each intent and what methods for evaluating them will be used in
Phase 2.
