#!/bin/bash

MANAGING_SUBSYSTEM=bt_mape_k

if [ "$1" == "gpu" ] || [ "$1" == "cpu" ];
then
    used_device=$1
else
    echo "Please provide the device you want to run this script on: gpu or cpu"
    exit 1
fi

if [ "${1:-}" = "cpu" ]; then
    USE_CPU=true
    docker_image="rossunset/sunset-artifact:cpu"
else
    USE_CPU=false
    docker_arg="--gpus=all"
    docker_image="rossunset/sunset-artifact:cuda"
fi

docker run ${docker_arg} -v ./log_dump:/home/dockuser/ros_ws/log_dump -v ./ros_ws:/ros_ws --rm --name mapek_bt ${docker_image} evaluation/scripts/run_single_experiment.sh baseline ${USE_CPU}


