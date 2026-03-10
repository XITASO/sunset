#!/bin/bash

# ------- Example resources --- #
#SBATCH --gres=shard:10
#SBATCH --job-name=sunset_run
# --- End Example resources --- #

handle_kill() {
    docker stop sunset_run
    docker rm sunset_run
    echo "Docker container stopped and removed."
    exit 0
}
trap 'handle_kill' SIGINT SIGTERM

./ros_ws/evaluation/example_run_docker.sh gpu

# Wait for the container to exit or a signal to be caught
docker wait sunset_run

# Cleanup after container finishes running normally
docker rm sunset_run
