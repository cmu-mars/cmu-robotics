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

The RR1 deliverable exists in three parts, one for each CP. They each live
in the top level directories bearing their name.

Inside each CP directory, you'll find:

* a directory `ta/` that's the result of running `swagger-codegen` on the
  Swagger definitions and then filling in the resulting stubs. Each has a
  README

* a simple script called `smoke.sh` to exercise each endpoint on each TA
  with some happy-path parameters, assuming it's running on localhost.
  
* a docker-compose.yml that specifies how to compose a docker container as
  a (potential) combination of several docker containers

An example of launching the system locally is as follows. Note that you can
parameterize the location of the TH and the TA by passing URI definitions to 
the docker compose command. In one terminal,

``` 
iev@bruce cp2 % TH_URI=http://th-brass:8080 TA_URI=http://ta-brass:8080 docker-compose up
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
iev@bruce cp2 % docker-compose down
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


