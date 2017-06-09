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
* Technologies for adapting software configurations and algorithms on-line

## Test Data

## Test Parameters

* Start and target location
* Initial battery level
* Hardware failure (e.g., (only?) kinect failure)
* (? Map)


## Test Procedure

The test procedure will be the same as for P1CP1, except that Lincoln labs will be able to perturb multiple times for each perturbation (e.g., place/remove obstacle, set battery, fail/reinstate kinect)

## Interface to the Test Harness (API)

Will mostly be the same as for P1CP1 with added API for introducing sensor failure.

## Intent Specification and Evaluation Metrics

* Accuracy, as per P1CP1
* Timeliness, as per P1CP1
* (not safety?)
