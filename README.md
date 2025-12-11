# Getting started with SUNSET

[[_TOC_]]

## Further documentation
For detailed information 
- about the system, how to evaluate experiments and how to extend the artifact, we refer to this [documentation](./system_description.md).
- about the managing system that is provided with this artifact and how you can integrate your own, we refer to this [documentation](./managing_systems.md)

## Requirements

- Docker (necessary)
- CUDA (optional if you want to run the segmentation models in the managed subsystem on the GPU)
- Docker nvidia runtime (optional)

If you want to run this artifact on the GPU, make sure that you have installed the nvidia container toolkit and configured docker to use the nvidia runtime (`/etc/docker/daemon.json`)
```
{
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    },
    "default-runtime": "nvidia"
}
```

Note: This has been thoroughly tested under WSL2 and native Ubuntu 22.04.
As everything is containerized in Docker, this should not be a problem as long as you have docker running and a CLI that supports `docker run/build` commands.

## Additional data
We use pretrained models and a ROS bag that replays the data which the pipeline uses.
All of the data are provided on a server. In the following are the instructions to setup the folder structure with the data in the right place:

1. Create a ".data" folder.
```bash
cd ros_ws && mkdir .data && cd .data
```

2. Download the segmentation models and the ros bags with test data from the a server with the scp command (password `seams`):


```bash
scp seams-reviewer@65.108.55.103:/home/seams-reviewer/data.zip .
unzip data.zip
```

In case unzip is not installed on your system:
```bash
sudo apt-get install unzip
```

In the end, the structure of your ros_ws should look like this:

```
ros_ws
├── .data
│   ├── checkpoints
│   │   ├── depth.pth
│   │   ├── fusion.pth
│   │   └── rgb.pth
│   └── SynDrone_t01_h50.bag
```

## Running the artifact

This artifact can be run without or with an exemplary managing system.
The logs will be stored in a folder `log_dump` next to the ros_ws directory.

Run the experiment from the root of the repository with one of the two following options (GPU or CPU). 
We recommend to use the GPU since there is a neural network running in SUNSET that runs smoother on the GPU.
### Running on GPU
```bash
bash ./ros_ws/evaluation/example_run_docker.sh gpu
```
### Running on CPU

```bash
bash ./ros_ws/evaluation/example_run_docker.sh cpu
```

## Setup in Dev Container
Create the necessary docker network:

```bash
docker network create sunset-network
```

There is a devcontainer file, i.e. you can just open VSCode in the root of this repository and reopen VSCode in the devcontainer. 
This should do the rest.

### Running on GPU
Note: By default we use the GPU to run our experiments. If you want to run experiments inside of the dev container, you can use the following script:

```bash
bash ./ros_ws/evaluation/example_run_local.sh gpu
```
The logs will be stored in a folder `log_dump` next to the ros_ws directory.

### Running on CPU
If you'd like to run them on your CPU make sure to remove the `-cuda` tag from the image in the [devcontainer.json](.devcontainer/devcontainer.json#L5) --> this will significantly impact the pace and therefore the reaction time and system down time of your results.

If you want to run experiments inside of the dev container, you can use the following script:
```bash
bash ./ros_ws/evaluation/example_run_local.sh cpu
```
Note: this will significantly impact the duration of your results.
The segmentation node will perform with a lower frequency which has impacts on the reaction time and system down time of your experiment.

