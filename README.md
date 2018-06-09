# cmu-robotics

As of 26 April, this file has been rewritten for how to build the
different CPs for RR2.

`documents` contains markdown files with draft descriptions of each
challenge problem following the provided format. The APIs describing the
interfaces for TA and TH in each CP case are given in
`documents/swagger-yaml` as valid [Swagger](http://swagger.io) files---with
automatically produced markdown also checked in for convenience.

All of these are currently still under revision and will change through the
process as we refine our research goals.

Inside each CP directory, you'll find at least:

* a directory `ta/` that's the result of running `swagger-codegen` on
  the Swagger definitions and then filling in the resulting stubs to
  mesh with the underlying subsystems.

* a `docker-compose.yml` that specifies how to composes our system
  with the LL TH image

* a `docker-compose-no-th.yml` that specifies a how to compose a
  docker container like the one above, but without the TH from Lincoln
  Labs

Each CP's TA follows the sequence diagram as described in this repo,
so it begins by sending a message to `/ready` on the TH and waiting
for a reply. Some of the TAs may have a fallback for reading a ready
message JSON object from the filesystem for debugging, but that is not
uniform.

## Phase II RR3 Instructions

### DockerHub Containers

For each challenge problem, we have tagged a container for integration week. The containers will allow you to run without being able to build. They are:

- cmumars/p2-cp1:RR2.IW
- cmumars/p2-cp2:RR2.IW
- cmumars/p2-cp3:RR3

To get these, you can simply do `docker pull cmumars/p2-cp3:RR3` for example. You will need to change the docker-compose files to refer to these builds though.

### Building CP1

1. Clone the CP1 base repo, `git clone
   git@github.com:cmu-mars/cp1_base.git` somewhere on the file system,
   which we'll call `CP1_BASE`.

<!-- 2. Clone the CP1 controllers repo, `git clone
   git@github.com:cmu-mars/cp1_controllers.git` somewhere on the file
   system, which we'll call `CP1_CONT`. -->

2. Build `cmu-mars/base` (note that this container is shared with CP3):

    ``` shell
    cd CMU_ROBOTICS/mars-main-p2
    docker build -t cmu-mars/base .
    ```

3. Build `cmu-mars/gazebo` (note that this container is shared with CP3):

    ``` shell
    cd CMU_ROBOTICS/cp-gazebo-p2
    docker build -t cmu-mars/gazebo .
    ```

4. Build `cmu-mars/cp1_base`:

    ``` shell
    cd CP1_BASE
    docker build -t cmu-mars/cp1_base .
    ```

<!-- 6. Build `cmu-mars/cp1_controllers`:

    ``` shell
    cd CP1_CONT
    docker build -t cmu-mars/cp1_controllers .
    ``` -->

5. Build `cmu-mars/cp1_rb`:

    ``` shell
    cd CMU_ROBOTICS/rainbow-p2
    docker build -t cmu-mars/cp1_rb -f Dockerfile-cp1 .
    ```

6. Build `cmu-mars/cp1`:

    ``` shell
    cd CMU_ROBOTICS/cp1/ta
    docker build -t cmu-mars/cp1 .
    ```

7. Compose `cmu-mars/cp1` with the TH:

    ``` shell
    cd CMU_ROBOTICS/cp1/ta
    TH_PORT=8081 TA_PORT=8080 docker-compose up
    ```

### Building CP2

Instructions for building and interacting with CP2 can be found at:
[cp2/ta/README.md](cp2/ta/README.md).

### Running CP3

1. Pull the docker image `cmumars/p2-cp3` from DockerHub
2. In the directory that you are wanting to compose in, ensure that the directorys `roslogs`, `logs` exist (these are where logs will be put), and ensure that they are Readable, Writable, and Executable for everyone.
3. Compose with:
``` shell
TA_PORT=8080 TA_PORT=8081 docker-compose -f docker-compose-mitll-harness.yml up
```

### Notes on limitations for CP3 in RR3

Some of the configurations do not have a plan associated with them on some paths, meaning that an A case will not run. If the TH tries to start one of these tests, it will get an error from the TA. There may also be some of these that exist during the actual evaluation. It is about 5% of the configuration/path space currently. We aim to reduce this.

### Building CP3
*Note*: Due to a last minute disappearance of one of the Unix packages (`ros-kinetic-mrpt-localization`) it is not possible to build CP3 from scratch. Use the DockerHub version instead. We are hoping this is temporary, but will develop a workaround before evaluation.


1. Clone the CP3 base repo, `git clone
   git@github.com:cmu-mars/cp3_base.git` somewhere on the file system,
   which we'll call `CP3_BASE`.

2. Build `cmu-mars/base` (note that this container is shared with CP1):

    ``` shell
    cd CMU_ROBOTICS/mars-main-p2
    docker build -t cmu-mars/base .
    ```

3. Build `cmu-mars/gazebo` (note that this container is shared with CP1):

    ``` shell
    cd CMU_ROBOTICS/cp-gazebo-p2
    docker build -t cmu-mars/gazebo .
    ```

4. Build `cmu-mars/cp3_base`:

    ``` shell
    cd CP3_BASE
    docker build -t cmu-mars/cp3_base .
    ```

5. Build `cmu-mars/cp3_rb`:

    ``` shell
    cd CMU_ROBOTICS/rainbow-p2
    docker build -t cmu-mars/cp3_rb -f Dockerfile-cp3 .
    ```

6. Build `cmu-mars/cp3`:

    ``` shell
    cd CMU_ROBOTICS/cp3/ta
    docker build -t cmu-mars/cp3 .
    ```

7. Compose `cmu-mars/cp3` with the TH:

    ``` shell
    cd CMU_ROBOTICS/cp3/ta
    TH_PORT=8081 TA_PORT=8080 docker-compose up
    ```

## Notes on getting this working in Windows 10

To run on Windows 10, you need to make sure that the correct port
forwarding is set up. So, as administrator, you may need to run:

```
netsh interface portproxy add v4tov4 listenaddress=127.0.0.1 listenport=8080 connectaddress=192.168.99.100 connectport=8080
```

Where `listenport` and `connectport` are the `TA_PORT` specified in
docker compose, and `connectaddress` is the IP of the host machine.
