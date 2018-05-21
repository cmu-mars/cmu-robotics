## Static Data for CP2

No static data is provided to CP2: the set of possible perturbations are
computed by the TA at run-time. The base image for the system under test
can be found at: https://github.com/cmu-mars/p2c2-base-image.


## Log Files

All of the components in CP2 should write their log files to the shared
`./logs/` directory that is mounted in each of the component containers as
specified by `docker-compose.yml`. If running on the host machine, `bugzood`
should also write its logs to the `./logs/` directory
(at `./logs/bugzoo/bugzood.log`, for example).
