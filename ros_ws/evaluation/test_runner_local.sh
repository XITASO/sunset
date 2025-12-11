#!/bin/bash

MANAGING_SUBSYSTEM=baseline

if [ "$1" == "gpu" ] || [ "$1" == "cpu" ];
then
    used_device=$1
else
    echo "Please provide the device you want to run this script on: gpu or cpu"
    exit 1
fi

if [ "$1" = "cpu" ]; then
    USE_CPU=true
else
    USE_CPU=false
fi

./evaluation/scripts/run_single_experiment.sh $MANAGING_SUBSYSTEM $USE_CPU

