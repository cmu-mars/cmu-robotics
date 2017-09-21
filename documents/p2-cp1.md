# CMU MARS (Aldrich), CP1: Integrated power model discovery and adaptation

## Overview

The goal of this challenge problem is to discover the power model of the
mobile robotics platform and use the discovered model to adapt for optimal
mission performance. The intent of this challenge problem is to simulate
robots with different hardware, algorithms, and workload and therefore
different power usage characteristics, in which we also intentionally simulate
severe changes that might be caused by unanticipated and yet-unknown
future environment changes. This challenge problem tests whether we can
adapt our robotic platform successfully to such situations.

This challenge problem is an extension of the MARS team's CP1 in the first
phase, "Low Power Adaptation." We expect the primary evaluation functions
and intent elements will remain the same.

Lincoln Labs will select a secret power model which is a function with a
large number of inputs (e.g. `20`) and outputs energy consumption over
time. That is, Lincoln Labs can provide drastic differences for energy
consumption of hardware and software components that may not reflect
current but possibly distant-future hardware and software. The MARS DAS
will be able to query the model by providing inputs and receiving back
the output from the power model. In addition, the power model is used
during the evaluation to compute the battery level during simulation.

The challenge problem will proceed in two phases. First, the DAS will use
the query mechanism to query the power consumption for certain inputs
(simulating the idea of running experiments in practice to measure power
consumption in specific configurations). The DAS will query only a small
number of inputs until a query budget (specified by Lincoln Labs) is reached.
The DAS will use the information gleaned to learn an approximation of the
secret power model.

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

## Research Questions
**RQ1**: Can the use of learning an accurate power model improve the score of mission compared with using inaccurate model and no model?
+ There might be some cases where using an accurate model might tell us that we can finish a mission without going to the charge station and therefore score better in the mission.
+ We would like to explore corner cases that an accurate model can provide us benefit by saving time, saving energy or both, and therefore hitting a better score in total.
+ Using an inaccurate model might tell us that we can go to the target but the discharge is quicker (t^2) than what the robot expects (t) and therefore fail the mission
- There might be some cases where the inaccurate model tells us we need to go to the charging station, but we could finish the mission without going to the station.

**RQ2**: Can the use of learning an accurate power model improve the quality of adaptations?
+ There might be some cases where an accurate model leads to the quality of the decisions made by the planner and analyzer. For example, an accurate model might trigger fewer adaptations, an accurate model might lead to better decision making by not going too much to the charging station or going only when it is needed.

**RQ3**: Can the use of a model that is accurate for a short horizon (i.e., t=[t_min, t_max/\alpha]) be beneficial for accomplishing a mission compared with a model that is more accurate for the longer horizon?

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

## Interface to the Test Harness (API)

### Notes about both REST APIs

Note, this API is notional at this stage.

* mode encodes: (pert|adaptation) & (No PM|Predefined PM|Learned PM)
* num_of_waypoints is the number of target points, i.e., sub-missions that
  needs to be completed mode is one of the following cases, cf. table below:
   1. A (no perturbation, no adaptation, no power model) so the robot does
   not have a clue to charge even when the battery goes below a threshold

   2. B (perturbation, no adaptation, no power model) so the robot does
   not have a clue to charge even when the battery goes below a threshold

   3. C (perturbation, adaptation, static predefined power model) so the
   planner uses an inaccurate model for planning an adaptation.  (We
   implicitly assume this is inaccurate.)

   4. C (perturbation, adaptation, learned model) that the planner use
   for adaptation

* The discharge and charge functions are what we mean by the power models.

  These are linear combinations of polynomial features with three different
  variables and interactions between them.

  More specifically, a power model is specified by the following general
   definition (formula): $f(o_1,\cdots,o_d) = \beta_0 + \sum_{o_i \in
   \mathcal{O}} \phi_i (o_i)
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

* Specific to CP1 are `PARSING_ERROR` and `LEARNING_ERROR` that may happen
  when something went wrong during the model parsing and model evaluation,
  or during the learning process, note that we have two separate packages
  for model parsing and evaluation as well as model learning. Model
  learning package use the parser to evaluate expression during learning,
  so the error might happen during the model evaluation (evaluating the
  expression of power model), or learning process.

### Formal specification of power models

Here, we formally define discharge and charge function, the constraints
that determine their validity, as well as metrics for evaluating the test
cases.

In this challenge problem, the possible variations (and possible adaptation
actions) determining the configuration of the robot is as follows:

1. *Robot's motion actuator*: Two levels of speed: s1 (half speed), s2
   (full speed)
2. *Robot's sensors*: Five different Kinects: k1 (the least expensive one),
   ..., k5 (the most expensive one)
3. *Robot's computation*: Five different localization algorithms ranging
   from the least computational demand for the most inaccurate localization
   to the most demand for the most accurate: l1 , ..., l5

Note that we abstracted different aspects of the robot that are known to be
the main source of power consumption in robots, i.e., robot's motion
actuator, sensors, and computationally intensive algorithms in the
robot. These variations will be implemented by adjusting different
measurement frequency of the Kinect sensor or spatial and depth resolution
of the base Kinect. Also, for the localization, we will implement different
components with different accuracy for localizing the robot.

Therefore, the configuration of the robot is encoded by 12 boolean
variables

```
C = [<s1,s2>,<k1,k2,k3,k4,k5>,<l1,l2,l3,l4,l5>]
```

At each time step in the simulation, one of the variables in the vector is
enabled and the rest are disabled. For example,

```
C_t = [<0,1>,<0,0,0,1,0>,<1,0,0,0,0>]
```

The total number of possible configuration for the robot is: `2*5*5 = 50`.

The power consumption model is then specified as:

```
P(t,s1,s2,k1,k2,k3,k4,k5,l1,l2,l3,l4,l5) = \beta_0 + \beta_s1*s1*f_s1(t) +
\beta_s2*s2*f_s2(t) + \beta_k1*k1*f_k1(t) + ... + \beta_k5*k5*f_k5(t) +
\beta_l1*l1*f_l1(t) + ... + \beta_l5*l5*f_l5(t) +
\beta_i1*s1*k1*l1*f_i1(t) + \beta_i2*s1*k1*l2*f_i2(t) + ... +
\beta_i_m*s2*k5*l5*f_i_m(t)
```

where t is in seconds, `s1,s2,k1,k2,k3,k4,k5,l1,l2,l3,l4,l5` are boolean
variables, the coefficients for the variables (\beta i) are any positive
real numbers and the coefficients for the interaction terms (\beta i j) are
any real numbers including negative or zero, (f i(t)) are non-negative
integer exponents of t. f i (t) - > [t,t^2,...,t^n], and the output of the
model after evaluation is in mWh.  Note that if we want to omit the effect
of any interactions, we simply put the corresponding coefficient to zero.

The interaction terms in the power consumption model are important. Let us
give an example for the necessity of capturing interactions in the power
model: If the robot is configured with a Kinect with higher accuracy, the
localization and other parts of the robots might need more computational
power to process the pixels, basically there is more information to
process, therefore, these two variables (i.e., Kinect and Localization)
might interact. This means that the consumption of the robot is bigger than
the consumptions of each of the Kinect and Localization individually.

Learning budget: `P(t,s1,s2,k1,k2,k3,k4,k5,l1,l2,l3,l4,l5)` is essentially
50 time dependent functions that are forged together. If there was no
interactions, we could assume that essentially the power model is an
additive model of time dependent of 12 terms. With the interactions, the
maximum number of terms is: 12 + C(12,2) + C(12,3) = 12 + 66 + 220 = 298. C
is binomial coefficient. Therefore, the required budget for learning is:

```
(the largest exponent of c1 + 1) + ... + (the largest exponent of c50 + 1)
```

In this challenge problem, both discharge and charge of the robot is
controlled, not by law of the physics for battery but as an arbitrary
function that looks similar to power model that exist int he literature but
with different coefficients that meant to simulate discharge and charge
functions for possible future sensory, computational, or actuating
components of the robot.

Here we define some constraints for the power consumption model. The
constraints will be used by LL and CMU team to evaluate whether an
specified power model is a valid power model:

1. P(t,s1,s2,k1,k2,k3,k4,k5,l1,l2,l3,l4,l5) > 0, if the power model is
   evaluated to be negative for a combination of input variables, then the
   power model is invalid.

2. P(t,s1,s2,k1,k2,k3,k4,k5,l1,l2,l3,l4,l5) should be monotonically
   increasing with respect to t.

Both of these constraints are intuitive for power models, because we do not
want to have a model that assumes the discharge operation actually
increases the charge of the battery instead of discharging it.

#### How to programatically evaluate whether a power model is invalid:

We can differentiate the power model, for all 50 possible configurations
separately, with respect to ``t'', if for all valid time
``t=[t_min,t_max]'' the sign of the derivative is positive, then the power
model is monotonically increasing. So, if for ``t=t_min'', and all
combination of valid configurations of the robot, ``P(t_min,.) > 0'', then
the power model is valid.

#### How the battery will be discharged and charged:

* *Discharge*: updated_charge = current_charge -
  P_discharge(dt,s1,s2,k1,k2,k3,k4,k5,l1,l2,l3,l4,l5)
* *Charge*: updated_charge = current_charge +
  P_charge(dt,s1,s2,k1,k2,k3,k4,k5,l1,l2,l3,l4,l5)

We also intend to specify some metrics based on which we evaluate how
"difficult" and how "similar" two test cases are. Therefore, LL could
generate "challenging" and yet "different" test cases, this is what we mean
by interesting test cases. The metrics are dependent on both specification
(including the shape of discharge/charge function and mission parameters)
of the mission as well as the perturbation during the mission.

#### The metrics for evaluating the difficulty of test cases:
1. Sum of the degree of the exponents of `t` for all terms in the power
   model

2. The number of obstacle placement + number of battery set. Any two test
   cases would be different if the difficulty levels of them are different.

The ultimate goal of CP1 is to demonstrate that the adaptation (analysis +
planning) with an accurate model that we learn is better than the case with
no learning, i.e, using an inaccurate model. So we assume the following
cases:

- A (no perturbation, no adaptation, no PM)
- B (perturbation, no adaptation, no PM)
- C (perturbation, adaptation, a static PM for discharge/charge, while
  planner uses a different static PM): the challenge with inaccurate model
- D (perturbation, adaptation, a PM will be specified by LL and planner
  uses a learned PM): the challenge with a learned model


So, an ideal situation for us is:

```
A: succeed, B: failed, C: failed, D: succeed
```

This situation is also good:

```
A: failed, B: failed, C: failed, D: succeed
```

### Sequence Diagram for Interaction Pattern

Implicitly, the TA can hit the `/error` endpoint on the TH at any time in
this sequence. This interaction is omitted for clarity.

<img src="sequences/cp1.mmd.png" alt="cp1 seq diagram" width="400">

### REST Interface to the TH

The Swagger file describing this interface is
[swagger-yaml/cp1-th.yaml](swagger-yaml/cp1-th.yaml) which should be
considered the canonical definition of the
API. [swagger-yaml/cp1-th.md](swagger-yaml/cp1-th.md) is produced
automatically from the Swagger definition for convenience.

This API is currently still a draft. Some, but not all, possible future
changes include:
 * adding more constants to the enumerated error codes in the TH `/error`
   end point

 * adding more constants to the enumerated status codes in the TH `/status`
   end point

The format `function-spec`, used to in the `/ready` end point to
describe the charge and discharge functions, is given by the following BNF:

```
polynomial ::= term | term "+" polynomial
term ::= factor | factor ops2 term | ops1"(" term ")"
factor ::= constant | variable | "(" polynomial ")"
variable ::= letter | variable digitSequence
constant ::= digitSequence | "-" digitSequence
digitSequence ::= digit | digit digitSequence
digit ::= "0" | "1" | "2" | ... | "9" | "e" | "PI"
letter ::= "s1" | "s2" | "k1" | ... | "k5" | "l1 ... "l5"
ops1 ::= "sqrt" | "exp" | "log" | "abs" | "-"
ops2 ::= "*" | "/" | "^"
```

Additionally, we require two semantic properties of the polynomials `f`
described with this syntax:

 1. _monotonicity_: For all times `t`, `df/dt(t) > 0`
 2. _positivity_: For all times `t`, `f(t) > 0`

### REST Interface to the TA

The Swagger file describing this interface is
[swagger-yaml/cp1-ta.yaml](swagger-yaml/cp1-ta.yaml) which should be
considered the canonical definition of the
API. [swagger-yaml/cp1-ta.md](swagger-yaml/cp1-ta.md) is produced
automatically from the Swagger definition for convenience.

This API is currently still a draft. Some, but not all, possible future
changes include:
 * adding more constants to the enumerated error codes in the `400` returns
   from different end points.



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
