# CMU MARS (Aldrich), CP2: Code-Level Adaptation

## Overview

In this document, we outline a challenge problem that requires code-level adaptation in response to source code perturbations. Our proposed challenge problem involves semi-automatically injecting code-level perturbations into the system, and alerting the code adaptation engine to the presence of a code-level perturbation (but not necessarily the location of the perturbation). We focus the scope of this challenge problem on perturbations that mimic the kinds of changes that are introduced through API migrations (e.g., method renaming, parameter addition, modification and removal, change of units, etc.).
 
 The process of injecting perturbations is to be conducted by a third-party (i.e., Lincoln Labs) with the aid of a perturbation engine, provided by ourselves. Below, we provide a high-level overview of the external evaluation process:

![alt text](img/brass-high-level.png "High-Level Architecture of the Perturbation Injection Process")

(1) The test harness sends a request to the perturbation engine, containing a description of the characteristics of the desired perturbation. (2) The perturbation engine generates a suitable perturbation at random, fitting the characteristics provided in step (1), and injects it into the source code. (3) The perturbation engine notifies the repair tool to the presence of a perturbation, invoking the process of code-level adaptation. (4) The code adaptation engine attempts to localise and adapt to the perturbation, within a given time or resource limit specified by the examiner. (5) The code adaptation engine provides a summary of the adaptation process to the test harness.

Using this approach, we propose that Lincoln Labs assesses the ability of the MARS system to self-adapt at the code-level by evaluating its response to a set of N generated code-level perturbation scenarios. To gain a more detailed understanding of the strengths and limitations of our code-adaptation engine, we propose that the system be evaluated against scenarios of varying levels of difficulty. For example, we may evaluate 60, 30 and 15 perturbation scenarios belonging to three coarsely defined difficulty levels D1, D2 and D3, respectively. Each of these scenarios is to be passed to the test harness to determine the system’s ability to respond.

## Test Data

No specific test data is required by this challenge problem. All code-level
perturbations will be generated using our supplied perturbation engine.

## Test Parameters

Below, we describe parameters that may be supplied to the perturbation engine
(via the test harness) to specify the nature of the perturbation.

**[INSERT TABLE]**

## Test Procedure

Below, we discuss each of the steps involved in the test procedure for this Challenge Problem:

1. **Generation:** A partial description of the perturbation scenario, provided
		by the examiner to the test harness, is forwarded onto the perturbation
		engine. The perturbation engine proceeds to generate a suitable code-level
		perturbation with the desired characteristics.
		</br></br>
		Generating realistic code-level perturbations, similar to those that are
		observed in software systems, remains an open challenge, not only for
		robotics but for software systems in general. The use of mutation tools and
		manual fault injection as a source of bugs for empirical studies has been
		shown to produce different results to when those studies are conducted
		using organic bugs [Pearson et al., 2017]. Careful consideration needs to
		go into the design of the perturbation engine to ensure realistic bugs are
		produced.

2. **Injection:** Once a suitable perturbation has been generated, that
		perturbation must be injected into the system. To reduce the complexity of
		modifying and recompiling the source code for the entire project, we
		propose the use of an alternative system deployment architecture: Instead
		of deploying the system to a single monolithic virtual machine, we propose
		that each logical component (e.g., navigation, planning, actuation) is
		deployed as a single, self-contained Docker container. System deployment is
		orchestrated using a single YAML file, hosted on a publicly visible Git
		repository, specifying logical components and the addresses of their
		associated images.
		</br></br>
		We propose to exploit the decentralised, ephemeral nature of this
		architecture to reduce the complexity of injecting code-level
		perturbations. Specifically, we plan to inject perturbations into the
		source code of the affected container, and recompile only that container,
		rather than recompiling the entire system.

3. **Validation:** The perturbed system is evaluated against a test suite to
		ensure that the system is sufficiently degraded, with respect to its
		intent. If the perturbation fails to result in sufficient degradation, it
		is discarded and an alternative perturbation is generated instead.
		(More details on “intent” and our evaluation metric can be found at a later
		section in this document).

4. **Adaptation:** Once a suitable perturbation has been injected, code-level
		adaptation is triggered. At this point, resource constraints (e.g.,
		wall-clock time, threads) and optional hints, describing the possible shape
		and location of the perturbation, are passed to the code adaptation engine.
		The code adaptation engine will attempt to find a code-level
		transformation that (partially) restores intent, within the specified
		resource limits. Once a suitable transformation has been found or resources
		have been exhausted, a summary of the repair trial is written to disk, as a
		JSON file. This summary may be accessed by the examiner through the
		test harness, described next.

## Interface to the Test Harness (API)

The test harness will be provided by a simple RESTful server. This server will
implement two actions: 1) perturbation injection and 2) perturbation status
checking. The perturbation injection action will use the parameters described
in the “Test Parameters” section to generate a perturbation scenario and
begin its evaluation, and will return a unique identifier for the scenario.
This identifier may be passed as an argument to the status checking action to
determine the current state of the evaluation for that perturbation scenario.
This action will return a “status” property, describing the state of the
scenario using one of several predefined labels. Each of these labels is
described below, together with any additional details that may be returned by
the API.

**[INSERT TABLE]**

To implement our perturbation engine, we plan to adapt and evolve our existing
perturbation injection tool for generic ROS systems, ROSHammer. ROSHammer is
currently capable of perturbing generic ROS systems at the architectural
level (e.g., adding noise to communications, dropping messages, killing nodes).
We currently use this functionality to increase the accuracy of model
inference (by falsifying hypotheses), although we envision its usage, by the
wider project, as a more general means of testing the adaptability of a system.
To perform mutations at the level of the code, we currently use another one of
our tools, Shuriken, a tool for generic program transformations in
large-scale programs. Shuriken currently targets C and C++, and can be used to
perform program repair, mutation testing, and fault localisation.
 
As part of the BRASS project and our wider research goals, we plan to
incorporate the abilities of Shuriken into ROSHammer. We envision that the
resulting tool will be capable of performing a wide range of adaptations, at
both the code and architectural levels. This tool may be used by any ROS-based
system to improve model inference, perform automated test generation, and to
systematically measure adaptiveness in a variety of scenarios.

## Intent Specification and Evaluation Metrics

To evaluate candidate code-level transformations, we propose that a set of
integration tests, each describing a mission for the robot (e.g., navigating a
simulated corridor), be performed in simulation. Instead of describing the
outcome of a mission as a success or failure, we describe outcomes in terms
of a set of predefined quality attributes (e.g., distance from the target,
power consumption, etc.). We define the intent of the system in terms of these
quality attributes: a system maintains intent if it completes a set of
missions to a satisfactory level of quality. If the system fails to meet this
expected level of quality, we deem it to be degraded. We define system quality
as an approximate measure of how closely the behaviour of a system meets its
intent. This definition allows us to recognise valuable adaptations that
partially restore intent (e.g., switching to an alternative, less accurate
navigation algorithm).

Each test case, or mission, is described by the following:

* a mission schema, describing a kind of mission. (e.g., navigate to a
	location.)
* a set of mission parameters, required to instantiate the mission schema as a
	concrete mission. (e.g., move from A to B.)
* a simulated environment. (e.g., a randomly-generated maze.)
* a configuration for the robot. (e.g., a certain node may be disabled.)
* a mission quality metric, defined by its schema, responsible for measuring
	the success of a mission.

This metric succinctly captures our goal for code-level adaptation: to return
a perturbed system as close to its intent as possible. From the perspective
of the code-level adaptation engine, this metric also transforms the problem
into one more amenable to search (i.e., it produces a gradient).

## References

[Pearson et al., 2017] Pearson, S., Campos, J., Just, R., Fraser, G., Abreu, R., Ernst,
M. D., Pang, D., and Keller, B. (2017). Evaluating and improving fault localization.
In Proceedings of the 2017 International Conference on Software Engineering, ICSE
’17. ACM. (To appear).
