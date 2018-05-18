# BRASS MARS, Phase II: Challenge Problem 2

## Installation

BugZoo needs to be installed onto the host machine. We recommend the use
of Python virtual environments to ensure containment and to prevent
interference with system libraries:

```
$ python3.6 -m venv PATH_TO_BUGZOO_VENV
$ . PATH_TO_BUGZOO_VENV/bin/activate
(bugzoo) $ pip install --upgrade bugzoo
(bugzoo) $ git clone git://github.com/cmu-mars/p2c2-base-image PATH_TO_BASE_IMAGE_REPO
(bugzoo) $ cd PATH_TO_BASE_IMAGE_REPO
(bugzoo) $ git checkout rr2
(bugzoo) $ bugzoo source add mars .
(bugzoo) $ bugzoo bug build --force mars:base
```

Finally, we need to build the images specified in the `docker-compose` file:

```
$ docker-compose build
```

## Usage

Before running `docker-compose`, you'll need to launch `bugzood` on the host
machine, as shown below. For the remainder of these instructions, we'll assume
that both `bugzoo` and `docker-compose` are running on the same machine
(ideally hosted on Amazon EC2).

```
(bugzoo) $ bugzood --host 0.0.0.0 --port 6060 --debug
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
