#!/bin/bash
#
# This file builds the base Docker image used by all CMU MARS Phase 1.5
# containers.
#
docker build -t cmu-mars/base       .
docker-compose build
