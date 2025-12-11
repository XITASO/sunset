#!/bin/bash

if [ "$2" == "true" ]; then
    echo "Running experiments on CPU"
else
    # Check if `nvidia-smi` command is available
    if ! command -v nvidia-smi &> /dev/null; then
        echo "NVIDIA drivers are not installed."
        exit 1
    fi
    # Check if NVIDIA drivers are currently running
    nvidia_smi_output=$(nvidia-smi)
    if [[ $? -ne 0 ]]; then
        echo "NVIDIA drivers are not running."
        exit 1
    fi
    nvidia-smi
fi

if [ "$1" == "baseline" ] || [ "$1" == "none" ];
then
    MANAGING_SUBSYSTEM=$1
else
    echo "Please submit one of the following options for a managing system: baseline, none"
    exit 1
fi


# make sure to read the changes
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install

source ./install/setup.bash


# Start experiment setup
ros2 launch experiment_setup experiment.launch.py &
PID1=$!
PGID1=$(ps -o pgid= -p $PID1 | tr -d ' ')

# Wait for the system to come to live
sleep 10
rate=0.2
# Start the ros bag
ros2 bag play .data/SynDrone_t01_h50.bag --clock  --rate=$rate&
PID3=$!
PGID3=$(ps -o pgid= -p $PID3 | tr -d ' ')

sleep 2

if [[ "$MANAGING_SUBSYSTEM" == "baseline" ]]; then
    echo "Starting managing subsystem: baseline"
    # Start the managing subsystem
    ros2 launch baseline baseline.launch.py &
    PID2=$!
    PGID2=$(ps -o pgid= -p $PID2 | tr -d ' ')
    sleep 2
fi



# Run scenario executor individually with specific scenario file
ros2 run experiment_setup scenario_executor seams_eval_example.yaml --ros-args -r __ns:=/experiment_setup -p use_sim_time:=true &
PID4=$!
PGID4=$(ps -o pgid= -p $PID4 | tr -d ' ')

# Define a function to kill the background processes
cleanup() {
    echo "Cleaning up..."
    # Attempt to kill gracefully first
    kill -TERM -- -$PGID1
    kill -TERM -- -$PGID2
    kill -TERM -- -$PGID3
    kill -TERM -- -$PGID4
}

# Set a trap to call the cleanup function on script exit
trap cleanup EXIT

# Simulate script work
echo "All processes have started..."
sleep 100

rm ~/parameters.csv
echo "Experiment finished."
