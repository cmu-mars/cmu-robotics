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

This readme will be expanded with instructions for running the system as we
being to produce artefacts in the run up to RR1.

RR1 Instructions
----------------

The RR1 deliverable exists in three parts, one for each CP. They each live
in the top level directories bearing their name.

Inside each CP directory, you'll find:

* a directory `ta/` that's the result of running `swagger-codegen` on the
  Swagger definitions and then filling in the resulting stubs. Each has a
  README

* a simple script called `smoke.sh` to exercise each endpoint on each TA
  with some happy-path parameters, assuming it's running on localhost.

An example of launching the system locally is as follows. In one terminal,

```
iev@bruce ta % docker build -t swagger_server . && docker run -p 8080:8080 swagger_server
Sending build context to Docker daemon  120.3kB
Step 1/9 : FROM python:3-alpine
 ---> a6beab4fa70b
Step 2/9 : RUN mkdir -p /usr/src/app
 ---> Using cache
 ---> cf9fb907b434
Step 3/9 : WORKDIR /usr/src/app
 ---> Using cache
 ---> 31b21fb96261
Step 4/9 : COPY requirements.txt /usr/src/app/
 ---> Using cache
 ---> 64f10e126ee6
Step 5/9 : RUN pip3 install --no-cache-dir -r requirements.txt
 ---> Using cache
 ---> 4c5ecd5e85ab
Step 6/9 : COPY . /usr/src/app
 ---> Using cache
 ---> b122e6a31f71
Step 7/9 : EXPOSE 8080
 ---> Using cache
 ---> 534e30b28a19
Step 8/9 : ENTRYPOINT python3
 ---> Using cache
 ---> 517480e3fec4
Step 9/9 : CMD -m swagger_server
 ---> Using cache
 ---> adeb961108e1
Successfully built adeb961108e1
Successfully tagged swagger_server:latest
 * Running on http://0.0.0.0:8080/ (Press CTRL+C to quit)
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
