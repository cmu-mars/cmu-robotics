#!/bin/bash -x

curl -i -X GET --header 'Accept: application/json' 'localhost:8080/observe'
curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "id" : "l1" , "state" : false }' 'localhost:8080/perturb/light'
curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "id" : "amcl" }' 'localhost:8080/perturb/nodefail'
curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "id" : "lidar" , "state" : false }' 'localhost:8080/perturb/sensor'
curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' 'localhost:8080/start'
