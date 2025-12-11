#!/bin/bash

# All combinations of managing system environment variables (true/false for each)
REPEAT=54

docker_arg=""
docker_image=""

if [ "$1" == "gpu" ] || [ "$1" == "cpu" ];
then
    used_device=$1
else
    echo "Please provide the device you want to run this script on: gpu or cpu"
    exit 1
fi

if [ "$1" = "cpu" ]; then
    USE_CPU=true
    docker_image="rossunset/sunset-artifact:cpu"
else
    USE_CPU=false
    docker_arg="--gpus=all"
    docker_image="rossunset/sunset-artifact:cuda"
fi

echo "Starting experiment..."

for i in $(seq 1 $REPEAT); do
    echo "Running experiment set $i..."
    docker run ${docker_arg} -v ./log_dump:/home/dockuser/ros_ws/log_dump -v ./ros_ws:/ros_ws --rm --name mapek_bt ${docker_image} evaluation/scripts/run_single_experiment.sh baseline ${USE_CPU}

    echo "experiment #$i finished. Preparing for next experiment..."
    sleep 5
done

echo "All experiments completed."
