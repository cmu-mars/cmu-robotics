#!/bin/bash
cat $ECS_CONTAINER_METADATA_FILE | jq -r '.TaskARN' | cut -d '/' -f2
