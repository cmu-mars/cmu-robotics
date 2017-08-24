#!/bin/bash

for f in *.mmd
do
    mermaid -w 500 $f
done
