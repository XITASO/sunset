#!/bin/bash

#  set args
MANAGING_SUBSYSTEM=bt_mape_k
USE_CPU=${1:-false}

docker_arg=""

# decide which Dockerfile to build and whether to enable GPUs
if [ "${1:-}" = "cpu" ]; then
    docker build . -t mapek_bt -f ./Dockerfile
    USE_CPU=true
else
    docker build . -t mapek_bt -f ./Dockerfile.cuda
    docker_arg="--gpus=all"
fi

echo "Build is done"

docker run ${docker_arg} -v ./log_dump:/home/dockuser/ros_ws/log_dump --rm --name mapek_bt_evaluation mapek_bt evaluation/run_single_experiment.sh ${MANAGING_SUBSYSTEM} ${USE_CPU}

echo "Fininshed docker run"
