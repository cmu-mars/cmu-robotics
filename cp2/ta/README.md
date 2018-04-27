# BRASS MARS, Phase II: Challenge Problem 2

## Installation

TODO: add BugZoo installation instructions

```
$ docker-compose build
```

## Usage

First, you'll need to launch BugZoo on a machine that's capable of provisioning
Docker containers:

```
> bugzood --host 0.0.0.0 --port 6060 --debug
```

Next, you'll need to use `docker-compose` to launch the TA, TH, and all of the
CP2 subsystems:

```
$ BUGZOO=http://host.docker.internal:6060 docker-compose up --build
```

where the `BUGZOO` environmental variable is used to specify the base URL of
the BugZoo server that you just launched. In the example above, the BugZoo
server is running on the same host as the Docker containers spawned by
`docker-compose`, so we can use `host.docker.internal` to access the host
from inside the Docker container.
