#!/bin/bash -x

curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' 'localhost:8080/start'
curl -i -X GET --header 'Accept: application/json' 'localhost:8080/observe'
curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "charge": 0  }' 'localhost:8080/perturb/battery'
curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "x" : 0 , "y" : 0  }' 'localhost:8080/perturb/place-obstacle'
curl -i -X POST --header 'Content-Type: application/json' --header 'Accept: application/json' -d '{ "obstacleid" : "obs1" }' 'localhost:8080/perturb/remove-obstacle'
