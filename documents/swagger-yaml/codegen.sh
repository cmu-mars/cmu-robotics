#!/bin/bash

for f in *-ta.yaml
do
    swagger-codegen generate -i $f -o ../../`basename -s '-ta.yaml' $f`/ta -l python-flask

    ## template files?
    ## assign type primitives?
    ## config file?
done
