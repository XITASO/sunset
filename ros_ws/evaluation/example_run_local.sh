#!/bin/bash

REPEAT=50

if [ "$1" == "gpu" ] || [ "$1" == "cpu" ];
then
    used_device=$1
else
    echo "Please provide the device you want to run this script on: gpu or cpu"
    exit 1
fi

echo "Starting experiments..."

if [ "$1" = "cpu" ]; then
    USE_CPU=true
else
    USE_CPU=false
fi

for i in $(seq 1 $REPEAT); do
    echo "Running experiment set $i..."

    ./evaluation/scripts/run_single_experiment.sh bt_mape_k ${USE_CPU}
    echo "experiment #$i finished. Preparing for next experiment..."
    sleep 5
done

echo "All experiments completed."
