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

## Phase II RR2 Instructions

The easiest way to see the difference in APIs since the RR1 build is
to use `git diff` with the appropriate commit SHA, i.e.

```
iev@iev-mbp swagger-yaml % git diff -w 7b2713e450425ead250d804a00012c599ad5da61 cp3-ta.yaml
diff --git a/documents/swagger-yaml/cp3-ta.yaml b/documents/swagger-yaml/cp3-ta.yaml
index fc1da1e..52b125c 100644
--- a/documents/swagger-yaml/cp3-ta.yaml
+++ b/documents/swagger-yaml/cp3-ta.yaml
@@ -87,9 +87,9 @@ paths:
               id:
                 type: string
                 enum:
-                  - kinect-ir
+                  - kinect
                   - lidar
-                  - kinect-all
+                  - camera
                 description: >-
                   which sensor of SENSORSET to set
               state:
@@ -138,10 +138,9 @@ paths:
               id:
                 type: string
                 enum:
-                  - movebase
                   - amcl
                   - mrpt
-                  - cb-base
+                  - aruco
                 description: >-
                   cause the named node to fail
       responses:
iev@iev-mbp swagger-yaml %
```

Below, we assume that this repo itself has already been cloned and
lives at `CMU_ROBOTICS` on the file system.

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

### Building CP3

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
