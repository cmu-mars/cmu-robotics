#!/bin/bash
## assume that we get the directory to copy as the first argument and the
## uuid as the second
echo "Executing command aws s3 cp $1 s3://dev-cmur-logs/$2/ --recursive"
aws s3 cp $1 s3://dev-cmur-logs/$2/ --recursive

##"aws", "s3", "cp", ld, "s3://dev-cmur-logs/" + config.uuid + "/" , "--recursive"
