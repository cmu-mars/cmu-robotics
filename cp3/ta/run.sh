#!/bin/sh

/home/haskell/cmur --thPort 5001 \
     --shellApi http://localhost:8080 \
     --taApi http://cp3_ta:5000/ \
     --parameters '{"baselineFlag": "ShouldSucceed", "testId": { "uuid": "6dc14858-6e9e-48b4-ba76-691ac505661a" }, "cp": "CP3", "testGrpId": "d625e0b1-6da7-4901-a496-7c165924b4ad", "params": {nodeFail : {"id" : "amcl"}, "cp3Init":  {"utility-function": "favor-timeliness", "start-configuration": "amcl-kinect", "start-loc": "l1", "use-adaptation": true, "target-loc": "l8"}, "lightState": {"state": false, "id": "l1"},  "sensorState": {"state": false, "id": "kinect"}}, "tcId": 0}'
