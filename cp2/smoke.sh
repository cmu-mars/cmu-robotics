#!/bin/bash -x

curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "attempt-limit" : 2 , "time-limit" : 20 }' 'localhost:8080/adapt'
curl -i -X GET --header 'Accept: application/json' 'localhost:8080/lines'
curl -i -X GET --header 'Accept: application/json' 'localhost:8080/observe'
curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "perturbations": [ { "kind": "DeleteStatement" } ] }' 'localhost:8080/perturb'
curl -i -X GET --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "file" : "main.cpp" , "line" : 500 , "shape" : "DeleteStatement" }' 'localhost:8080/perturbations'
