# BRASS MARS, Phase II: Challenge Problem 2

## Installation

To download the requisite images for this challenge problem and to build the
image for the TA:

```
$ ./install
```

## Usage

```
$ docker-compose up
```

For the purposes of debugging, logs for the various components involved in CP2
should be written to the `./logs` directory. The `./logs` directory should not
be mounted during deployment. (Instead, an ephemeral `logs` volume should be
shared between the containers -- see "Logging Verbosity" for more details.)

## Deployment

We have tested this challenge problem on Amazon EC2 using the "Ubuntu Server
16.04 LTS (HVM), SSD Volume Type" AMI (`ami-6a003c0f`) running on a number of
different instance types.

### Instance Type

Specify which instance type should be used.

### Resource Limits

In line with established practices for search-based repair techniques within
the automated program repair literature, CP2 should be given ten hours to
attempt to find a complete repair to its perturbation. No limit should be
placed on the number of candidate patches that may be evaluated during that
time.

### CPU Resources

To ensure the best outcome and to exploit the full potential of its underlying
technology, CP2 needs to run on a machine with a large number of CPU cores.
The `--threads` option to the TA is used to control the number of (logical)
CPU threads that are allocated to the challenge problem. For the sake of
predictability, the `--threads` option should be set to the number of threads
that are available to the underlying machine. (For some perturbations, the
repair system can run with 3--4X the number of logical threads since those
threads spend most of their time sleeping, but as that is not universally
true, we don't recommend doing that.)

### Memory Limits

Appropriate memory limits for the various containers are given both below and
in `docker-compose.yml`.

* *BugZoo:* 1GB
* *Boggart:* 500MB
* *Rooibos:* 500MB
* *TA:* 8GB

In total, 10GB of memory should be allocated to the *core* service containers
for CP2. The remaining memory on the instance will be used by dynamically
allocated containers that are used to perform patch evaluation. The amount of
memory provided by the instance types described above should be more than
sufficient to cover the dynamic memory needs of this CP.

### Logging Verbosity

Each of the services defined in `docker-compose.yml` (with the exception of
`rooibos`) writes to its own log file and come with options for controlling
logging verbosity.

* `bugzoo`: the logging level is set via `--log-level` option and may be set
  to any of the following: `none`, `info`, `error`, `warning`, `debug`, or
  `critical`. Given the large number of calls to `bugzoo` over the course of
  the evaluation, we recommend logging at the `warning` level during
  deployment. For debugging, the level can be dropped to `info` or `debug`
  (but be warned that `debug` can potentially produce GBs of logs).
* `boggart`: implements an identical logging system `bugzoo`, but produces
  relatively small logs. We recommend using the `info` level for deployment
  and `debug` for debugging.
* `rooibosd`: logging for this component should be **completely disabled** by
  redirecting its output to `/dev/null`. Any failures in this component
  should be captured by client logs in the other components.
* `ta`: the TA does not expose any of its own logging options.

Before posting to `/done` or `/error`, the TA will attempt to collect logs
from all of the containers above, before aggregating them into a compressed
archive (i.e., a `.tar.gz` file) and uploading the resulting archive to Amazon
S3.

## Troubleshooting

* **`docker-compose` has stopped writing to stdout:** for an unknown reason,
  `docker-compose` stops writing the the stdout when the Amazon Linux AMI is
  used. The issue does not occur with the Ubuntu Server AMI.


## Guidance

* It takes a long time to generate coverage for perturbations to files in
  `rostime`, `roslib`, or any other core ROS library, since those files are
  covered by all systems tests and integration tests (as well as a number of\
  unit tests).
