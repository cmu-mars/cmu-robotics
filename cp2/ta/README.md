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

## Troubleshooting

* **`docker-compose` has stopped writing to stdout:** for an unknown reason,
  `docker-compose` stops writing the the stdout when the Amazon Linux AMI is
  used. The issue does not occur with the Ubuntu Server AMI.
