#!/bin/bash

for f in *.yaml
do
    swagger-codegen validate -i $f
done
