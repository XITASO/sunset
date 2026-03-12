# Managing system

## Exemplary managing system

We provide a baseline that acts as exemplary managing system which is placed in [this ros package](./ros_ws/src/managing/baseline).
It redeploys every ROS node that has a low frequency and recalibrates the sensor fusion if the entropy of the segmentation model is too high. 
This is only one of the viable options for the symptom of high entropy.

## Integrate your own managing system

This repository ships one managing subsystem explained [here](#exemplary-managing-system). You can add your own manager (rule‑based, BT‑driven, learning, heuristic, etc.) and still reuse the evaluation + logging pipeline.

### 1. Where to put your code
Create a new ROS 2 package under:
```
/ros_ws/src/managing/<your_manager_pkg>/
```

Add in your `package.xml` the dependencies you actually use (e.g. `behaviortree_cpp_v3`, `ament_cmake`, `std_msgs`).

### 2. Minimum integration contract
Your manager should (at least) be able to:
- Subscribe to the system status / sensor or degradation topics you care about (see existing packages for examples).
- Compute an adaptation decision (strategies / actions) when triggers fire.
- (Optional but recommended) Publish `ExperimentLogging` messages (`system_interfaces/msg/ExperimentLogging.msg`) so it’s captured by the existing evaluation scripts.

If you want parity with the reference manager’s logging granularity, also use:
- `StrategyStatus.msg` for per‑strategy lifecycle (triggered / success / failure).
- `AdaptationStatus.msg` & `GenericAdaptation.msg` when you execute concrete adaptations.

Inspect `ros_ws/src/system_interfaces/msg/` for all available message definitions.

### 3. Making the scripts aware of your manager
Update these scripts if needed:
- Local single run: `ros_ws/evaluation/scripts/run_single_experiment.sh`
- Batch / example runs: `ros_ws/evaluation/example_run_local.sh`, `example_run_docker.sh`
- Docker single run (if you use the new flag interface): `ros_ws/evaluation/docker_single_experiment.sh`

Add your package name as an allowed value for `MANAGING_SUBSYSTEM` and any specialized env vars you require.

### 4. Logging integration (recommended)
To have your events captured in the CSV files under `log_dump/`, publish `ExperimentLogging` messages on `/bt_executor_log`; each message is appended as one row. 
To log additional sources (e.g. a new node or manager), add a subscription entry to the `comm_types` list in `ros_ws/src/experiment_setup/experiment_setup/config/experiment_logger_config.py`. Each entry needs: `comm_type=CommunicationTypes.SUBSCRIPTION`, the topic `name`, `msg_type=ExperimentLogging`, and `callback="logger_callback"`. After restarting the experiment the new topic’s events will appear in the CSV.

`ExperimentLogging.msg` fields:

| Field | Type | Meaning |
|-------|------|---------|
| `timestamp` | `uint64` | Monotonic timestamp (nanoseconds) when the log event was created. |
| `scenario` | `string` | Scenario identifier (e.g. `_001` from scenario executor). Empty for pure metric ticks. |
| `source` | `string` | Component emitting the log (`/evaluator_log`, `analyzer`, `planning`, `execution_redeploy`, node-specific). |
| `rule_name` | `string` | Name of the rule currently being evaluated / triggered (blank when not in a rule context). |
| `strategy_name` | `string` | Selected strategy name (blank until planning picks one). |
| `strategy_hash` | `uint64` | Stable hash of the strategy (used to correlate lifecycle events). 0 if none. |
| `strategy_status` | `uint8` | Status code from `StrategyStatus.msg` (e.g. triggered=1, finished success=5). |
| `adaptation_type` | `int8` | Numeric code of adaptation (matches `AdaptationType.msg`; e.g. restart, redeploy). 0 if not in adaptation phase. |
| `adaptation_status` | `uint8` | Status code from `AdaptationStatus.msg` (1 triggered, 2 running, 3 finished). 0 if N/A. |
| `success` | `bool` | Whether the last adaptation / strategy concluded successfully (True only after finalization). |
| `iou` | `float32` | Global segmentation Intersection-over-Union at this tick. 0.0 if not computed. |
| `vehicle_iou` | `float32` | Class-specific IoU (vehicle) at this tick. |
| `road_iou` | `float32` | Class-specific IoU (road) at this tick. |
| `gt_failure_name` | `string` | Ground-truth failure label injected (e.g. `shutdown`, `image_degradation`). Empty if none. |
| `is_gt_failure` | `bool` | True if this row marks the ground-truth failure event itself. |

Keep unused fields at default values (0 / empty string / False). Downstream scripts rely on column order, not presence of every semantic.

### 5. Docker considerations
If your manager adds external dependencies:
- Extend `Dockerfile` / `Dockerfile.cuda` with the required apt / pip installs.
- Pass the correct image name into your run scripts or add a conditional on `MANAGING_SUBSYSTEM`.

### 6. Communication
To receive and to react to data from the managed system it is recommended to use the `abstract_blackboard_setter`. This node collects data from the managed system and relays it to the managing system, acting as a middle ware, such that no additional publishers or subscribers are needed. 
By default it listens to the `/diagnostics` - topic, relays the status and calculates the heartbeat frequency for each node. If addional data is needed, the `abstract_blackboard_setter` can be [configured](./ros_ws/src/base/managed_subsystem/resources/bbs_wiring.json)., to also include data from arbitrary ROS topics. Note however that only int, float, bool, and string type data is supported.
An example configuration is given below:

```
{
    "/managed_subsystem/is_image_degraded":[
        "is_degraded"
    ],
}
```
In this case the value `is_degraded` of messages sent on the topic `/managed_subsystem/is_image_degraded` is forwarded to the managing system. 
The names of the sent parameters are equal to the names of the values; in this example the parameter name would just be `image_degaded`, However, if the same value name occurs in multiple topics, ubique parameter names are ensured, by expanding them by the topic name. In this example the parameter name would be `managed_subsystem/is_image_degraded/is_degraded"`.

With this in place you can swap managers simply by changing `MANAGING_SUBSYSTEM` in the scripts (or using a new CLI flag if you extend the docker runner).

