# Test Design (CP2)

Each test should inject a single perturbation into the baseline system.
The set of possible perturbations should be computed dynamically using the
TA's API. Changes to the source code that do not result in at least one test
failure are considered invalid and are reported as such by the TA.

To ensure a fair assessment of the performance of our system, the following
coverage criteria should be considered:

* file coverage: perturbations should be uniformly distributed across the
    different files (n.b. the list of files can be obtained using the TA API).
* operator coverage: each of the different types of operator should be
    covered.

At the end of the scenario, the TA should report a summary of the outcome of
the adaptation process. Below are the three possible outcomes of the
adaptation:

* **Complete repair:** a code adaptation was found that results in all of the
    tests passing.
* **Partial repair:** a code adaptation was found that results in the passing
    of at least one previously failing test but not all previously failing
    tests.
* **No repair:** no code adaptation could be found that causes any previously
    failing 
