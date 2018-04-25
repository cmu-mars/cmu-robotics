cmu-robotics
============

As of 18 August, this repo has been cleaned of Phase 1 content following
[git_process.md](https://github.mit.edu/brass/mitll/blob/master/documents/development/git_process.md).

`documents` contains markdown files with draft descriptions of each
challenge problem following the provided format. The APIs describing the
interfaces for TA and TH in each CP case are given in
`documents/swagger-yaml` as valid [Swagger](http://swagger.io) files---with
automatically produced markdown also checked in for convenience.

All of these are currently still under revision and will change through the
process as we refine our research goals.

Phase II RR1 Instructions
----------------

The RR1 deliverable exists in five parts: one docker container for each CP and
a docker container for the base ROS and Gazebo containers. They each live
in the top level directories bearing their name.

To build each docker container, the following process needs to be followed:

```
$ cd mars-main-p2
$ docker build -t cmu-mars/base .
$ cd ../cp-gazebo-p2
$ docker build -t cmu-mars/gazebo .
$ cd ../cp1/ta
$ docker build -t cmu-mars/cp1 .
$ cd ../../cp2/ta
$ docker build -t cmu-mars/cp2 .
$ cd ../../cp3/ta
$ docker build -t cmu-mars/cp3 .
```

Once the containers are built, it will be possible to compose them
for each challenge problem.

Inside each CP directory, you'll find:

* a directory `ta/` that's the result of running `swagger-codegen` on the
  Swagger definitions and then filling in the resulting stubs. Each has a
  README

* a simple script called `smoke.sh` to exercise each endpoint on each TA
  with some happy-path parameters, assuming it's running on localhost.

* a `docker-compose.yml` that specifies how to compose a docker container as
  a (potential) combination of several docker containers, including the th
  from Lincoln Labs

* a `docker-compose-no-th.yml` that specifies a how to compose a docker container
  like the one above, but without the TH from Lincoln Labs

An example of launching the system locally is as follows. Note that you can
parameterize the location of the TH and the TA by passing URI definitions to
the docker compose command. In one terminal, in the `ta` directory,

```
iev@bruce ta % TH_PORT=8081 TA_PORT=8080 docker-compose up
Creating network "ta_default" with the default driver
Creating roscore ... <Note that the actual containers composed will differ between CPs>
Creating roscore ... done
Creating gazebo ...
Creating gazebo ... done
Creating cp3_ta ...
Creating cp3_ta ... done
Attaching to roscore, gazebo, cp3_ta
... <TRACE MESSAGES FROM THE CONSOLE>
```

To stop the instance, use the command:

```
iev@bruce ta % docker-compose down
```

In another terminal, run the relevant smoke script:

```
iev@bruce cp2 % clear && ./smoke.sh

+ curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "attempt-limit" : 2 , "time-limit" : 20 }' localhost:8080/adapt
HTTP/1.0 200 OK
Content-Type: application/json
Content-Length: 3
Server: Werkzeug/0.12.2 Python/3.6.2
Date: Sun, 08 Oct 2017 23:14:28 GMT

{}
+ curl -i -X GET --header 'Accept: application/json' localhost:8080/lines
HTTP/1.0 200 OK
Content-Type: application/json
Content-Length: 54
Server: Werkzeug/0.12.2 Python/3.6.2
Date: Sun, 08 Oct 2017 23:14:28 GMT

[
  {
    "file": "main.cpp",
    "number": 500
  }
]
+ curl -i -X GET --header 'Accept: application/json' localhost:8080/observe
HTTP/1.0 200 OK
Content-Type: application/json
Content-Length: 501
Server: Werkzeug/0.12.2 Python/3.6.2
Date: Sun, 08 Oct 2017 23:14:28 GMT

{
  "pareto-set": [
    {
      "degradation": {},
      "diff": "1c1\n< a\n---\n> b\n",
      "test-outcomes": [
        {
          "crashed": false,
          "qos": {
            "collisions": {},
            "duration": {},
            "proximity": {}
          },
          "test-id": "a",
          "time-taken": "509",
          "timed-out": false
        }
      ]
    }
  ],
  "resource-consumption": {
    "num-attempts": 5,
    "time-spent": 43.2
  },
  "stage": "awaiting-perturbation"
}
+ curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "perturbations": [ { "kind": "DeleteStatement" } ] }' localhost:8080/perturb
HTTP/1.0 200 OK
Content-Type: application/json
Content-Length: 3
Server: Werkzeug/0.12.2 Python/3.6.2
Date: Sun, 08 Oct 2017 23:14:28 GMT

{}
+ curl -i -X GET --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "file" : "main.cpp" , "line" : 500 , "shape" : "DeleteStatement" }' localhost:8080/perturbations
HTTP/1.0 200 OK
Content-Type: application/json
Content-Length: 72
Server: Werkzeug/0.12.2 Python/3.6.2
Date: Sun, 08 Oct 2017 23:14:28 GMT

{
  "perturbations": [
    {
      "kind": "swap arguments"
    }
  ]
}
iev@bruce cp2 %

```

To compose withouth a th (although, you still need to have a TH somewhere), you should do the following:

```
iev@bruce ta % TH_HOST=<some host> TH_PORT=8081 TA_PORT=8080 docker-compose up
```
As part of the composition, each docker instance will contain a docker container
`cp<N>_ta` where `<N>` is the number of the challenge problem. You may log into
this container, in particular to access the log file that contains the details of
each request made to the TA, as information about call attempts to the TH. For example,
for challenge problem 3:

```
iev@bruce cp3 % docker exec -it cp3_ta bash
mars@cp3_ta:/usr/src/app$ cat access.log
Failed to connect with th
Sending ready
Fatal: could not connect to TH -- see last logger entry to determine which one
Starting TA
 * Running on http://0.0.0.0:8080/ (Press CTRL+C to quit)
mars@cp3_ta:/usr/src/app$ exit
exit
```

What happens when you run
-------------------------

This release is purely for testing API compliance and the ability to build in the LL environment. As such, none of the APIs have semantics attached to them, neither do we assume semantics from the TH. When the container is run, the following things happen:

1. (In cp1, cp2, and cp3) ROS starts up (caused by starting the cmu-mars/main container in mars-main-p2)
2. (In cp1 and cp3) Gazebo starts up (caused by starting the cmu-mars/gazebo conainer in cp-gazebo-p2)
3. (In cp1, cp2, and cp3) We post a dummy error, with the error "Test Error" to the TH
4. (In cp1 and cp3) We try to conneect to Gazebo, and post a "Gazebo Error" to the TH if this fails
5. (In cp2, cp2, and cp3) We start the TA, at which time the TH can test our interface
6. (In cp1, cp2, and cp3) We post ready, status, and (after a time ranging between 5 and 60 seconds) done messages to TH. We print out the return to ready. We log exceptions if we fail on other calls.

During the process, we log all calls sent and received to access.log in the cmu-mars/cp<N> container.

## Notes on getting this working in Windows 10

To run on Windows 10, you need to make sure that the correct port forwarding is set up. So, as administrator, you may need to run:

```
netsh interface portproxy add v4tov4 listenaddress=127.0.0.1 listenport=8080 connectaddress=192.168.99.100 connectport=8080
```

Where `listenport` and `connectport` are the `TA_PORT` specified in docker compose, and `connectaddress` is the IP of the host machine.

Phase II RR2 (Early Integration) Instructions
----------------

As of 16 April, CP3 is the only CP that's been integrated with its
TA. The build process is as follows:

```
# in a clone of this repository
cd mars-main-p2
docker build -t cmu-mars/base .
cd ../cp-gazebo-p2
docker build -t cmu-mars/gazebo .
# in a clone of the cp3_base repository, develop branch
docker build -t cmu-mars/cp3_base .
# back in cmu-mars clone
cd ../cp3/ta
docker build -t cmu-mars/cp3 .
TH_PORT=8081 TA_PORT=8080 docker-compose up

```

The compose file currently has the dependency to the TH commented out,
per https://github.mit.edu/brass/mitll-harness/issues/19

The easiest way to see the difference in APIs since the RR1 build is
to use `git diff` with the appropriate commit SHA:

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
