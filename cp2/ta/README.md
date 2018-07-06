# BRASS MARS, Phase II: Challenge Problem 2

## Installation

```
$ ./install
```

## Usage

```
$ docker-compose up
```

Logs for the various components involved in CP2 should be written to the
`./logs` directory.


## Deployment

We have tested this challenge problem on Amazon EC2 using the "Ubuntu Server
16.04 LTS (HVM), SSD Volume Type" AMI (`ami-6a003c0f`) running on a number of
different instance types. The `--threads` flag supplied to the TA inside
`docker-compose.yml` should be adjusted to match the compute resources
provided by the instance.


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

TODO: provide guidance on appropriate logging levels for deployment.

Before posting to `/done` or `/error`, the TA will attempt to collect logs
from all of the containers listed in `docker-compose.yml`, before aggregating
them into a compressed archive (i.e., a `.tar.gz` file) and uploading the
resulting archive to Amazon S3.


## Troubleshooting

* **`docker-compose` has stopped writing to stdout:** for an unknown reason,
  `docker-compose` stops writing the the stdout when the Amazon Linux AMI is
  used. The issue does not occur with the Ubuntu Server AMI.


## Guidance

* It takes a long time to generate coverage for perturbations to files in
  `rostime`, `roslib`, or any other core ROS library, since those files are
  covered by all systems tests and integration tests (as well as a number of\
  unit tests).
