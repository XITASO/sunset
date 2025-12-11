from rclpy.node import Node
import rclpy
from system_interfaces.msg import SetBlackboardGroup, ExperimentLogging, StrategyStatus
from dataclasses import dataclass
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import subprocess
from rcl_interfaces.msg import ParameterType, Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
import time

@dataclass
class Adaptation:
    queued_for_execution: bool = False
    node_to_adapt : str = ""

class BaselineManagingSystem(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.bb_subscription = self.create_subscription(SetBlackboardGroup, "blackboard_group", self._monitor, 10)
        self.exp_logger = self.create_publisher(ExperimentLogging, "/experiment_setup/bt_executor_log", 10)
        self.knowledge_base = {"need_autofocus": False}
        
        # If counter is needed
        self.system_state_mem = {
            "redeploy_camera": 0,
            "redeploy_depth": 0,
            "redeploy_sensor_fusion": 0,
            "redeploy_segmentation": 0,
            "do_recalibration": 0,
            "camera_autofocus": 0
        }
        self.adaptations = {
            "redeploy_camera": Adaptation(False, "/managed_subsystem/camera"),
            "redeploy_depth": Adaptation(False, "/managed_subsystem/depth"),
            "redeploy_sensor_fusion": Adaptation(False, "/managed_subsystem/sensor_fusion"),
            "redeploy_segmentation": Adaptation(False, "/managed_subsystem/segmentation"),
            "do_recalibration": Adaptation(False, "/managed_subsystem/sensor_fusion"),
            "camera_autofocus": Adaptation(False, "/managed_subsystem/camera"),
        }
        self.evaluate_dict = {
            "managed_subsystem_rgb_camera_freq":{
                "operator": "<",
                "value": 1.
            },
            "managed_subsystem_depth_camera_freq":{
                "operator": "<",
                "value": 1.
            },
            "managed_subsystem_sensors_fused_freq":{
                "operator": "<",
                "value": 1.
            },
            "managed_subsystem_segmentation_freq":{
                "operator": "<",
                "value": 1.
            },
            "segmentation_entropy":{
                "operator": ">",
                "value": 0.06
            },
            "autofocus_needed":
            {
            }
        }
        self.create_timer(0.2, self._mapek_loop)

    def _mapek_loop(self):
        self._analyze()
        self._plan()
        self._execute()
        self.get_logger().info("MAPE-K Loop executed")

    def _monitor(self, msg: SetBlackboardGroup):
        self.get_logger().info("Monitoring executed")
        for param in msg.bb_params:
            if param.value.type == ParameterType.PARAMETER_DOUBLE:
                self.knowledge_base[param.name] = float(param.value.double_value)
            elif param.value.type == ParameterType.PARAMETER_BOOL:
                self.knowledge_base[param.name] = param.value.bool_value

    def _evaluate_system_state(self, key: str, value = None, operator: str = "<") -> bool:
        """
        Evaluate system state based on knowledge base values.
        
        Args:
            key: The key in the knowledge base to evaluate
            value: The value to compare against (for numeric comparisons)
                   Can be float for numeric comparisons or bool for boolean checks
            operator: The comparison operator to use ("<" or ">")
                     Only used for numeric (double) comparisons
        
        Returns:
            bool: True if the condition is met, False otherwise
        
        Raises:
            ValueError: If the operator is not "<" or ">"
        """
        if key not in self.knowledge_base:
            return False
        
        kb_value = self.knowledge_base[key]
        
        # Handle boolean comparisons
        if isinstance(kb_value, bool):
            # For boolean values, value parameter should also be bool or omitted
            if value is None:
                return kb_value
            elif isinstance(value, bool):
                return kb_value == value
            else:
                return False
        
        # Handle numeric (double) comparisons
        if isinstance(kb_value, (int, float)):
            if value is None:
                return False
            
            # Validate operator
            if operator not in ("<", ">"):
                raise ValueError(f"Unsupported operator: {operator}. Only '<' and '>' are supported.")
            
            try:
                # Convert value to float for comparison
                float_value = float(value)
                if operator == "<":
                    return kb_value < float_value
                elif operator == ">":
                    return kb_value > float_value
            except (TypeError, ValueError):
                return False
        
        return False

    def _analyze_system_state(self, event: str, knowledge_base_key: str):
        if self._evaluate_system_state(knowledge_base_key,
                                       value=self.evaluate_dict[knowledge_base_key]["value"] if "value" in self.evaluate_dict[knowledge_base_key] else None,
                                        operator=self.evaluate_dict[knowledge_base_key]["operator"] if "operator" in self.evaluate_dict[knowledge_base_key] else None):
            if self.system_state_mem[event] > 0:
                pass
            else:
                self.get_logger().info(f"Appended event {event}")
                self.events.append(event)
        elif self.system_state_mem[event] > 0:
            self.system_state_mem[event] = 0
            self._log_experiment(source="analyzer", rule_name=event, strategy_name=event, strategy_status=StrategyStatus.STATUS_STRATEGY_FINISHED_SUCCESSFUL)

    def _analyze(self):
        self.events = list()

        event_key_map = {
            "redeploy_camera": "managed_subsystem_rgb_camera_freq",
            "redeploy_depth": "managed_subsystem_depth_camera_freq",
            "redeploy_sensor_fusion": "managed_subsystem_sensors_fused_freq",
            "redeploy_segmentation": "managed_subsystem_segmentation_freq",
            "do_recalibration": "segmentation_entropy",
            "camera_autofocus": "autofocus_needed",
        }
        for event, knowledge_base_key in event_key_map.items():
            self._analyze_system_state(event, knowledge_base_key)

        for event in self.events:
            self.adaptations[event].queued_for_execution = True
            self.system_state_mem[event] = 5
            self._log_experiment(source="analyzer", rule_name=event)
        time.sleep(1)
        

    def _plan(self):
        for event, adaptation in self.adaptations.items():
            if adaptation.queued_for_execution == True:
                self._log_experiment(source="planning", strategy_name=event, strategy_status=StrategyStatus.STATUS_STRATEGY_TRIGGERED)

    def _log_experiment(self, source: str="", rule_name: str = "", strategy_name: str = "", strategy_status: int=None):
        msg = ExperimentLogging()
        msg.timestamp = self.get_clock().now().nanoseconds
        msg.source = source
        msg.rule_name = rule_name 
        msg.strategy_name = strategy_name
        if strategy_status is not None:
            msg.strategy_status = strategy_status

        self.exp_logger.publish(msg)

    def _execute(self):
        for key, value in self.adaptations.items():
            if value.queued_for_execution == False:
                continue
            if "redeploy" in key:
                self._perform_redeploy(value.node_to_adapt)
                value.queued_for_execution = False
            elif "calibration" in key:
                param = Parameter()
                param.name = "do_recalibration"
                param_value = ParameterValue()
                param_value.bool_value = True
                param_value.type = ParameterType.PARAMETER_BOOL
                param.value = param_value
                
                self._perform_parametrization(value.node_to_adapt, param)
                value.queued_for_execution = False
            elif "autofocus" in key:
                param = Parameter()
                param.name = "perform_autofocus"
                param_value = ParameterValue()
                param_value.bool_value = True
                param_value.type = ParameterType.PARAMETER_BOOL
                param.value = param_value
                
                self._perform_parametrization(value.node_to_adapt, param)
                value.queued_for_execution = False

    def _perform_parametrization(self, node_name: str, parameter: Parameter):
        client = self.create_client(SetParameters, f"{node_name}/set_parameters")
        req = SetParameters.Request()
        req.parameters.append(parameter)
        self.future = client.call_async(req)

    def _perform_redeploy(self, node_name: str):
        client = self.create_client(ChangeState, f"{node_name}/change_state")
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVE_SHUTDOWN
        self.future = client.call_async(req)

        node = node_name.split("/")[2]

        subprocess.Popen(["ros2", "launch", "managed_subsystem", f"{node}.launch.py"])
        self.get_logger().info("Performed redeploy")


def main(args=None):
    rclpy.init(args=args)
    node = BaselineManagingSystem("baseline")
    rclpy.spin(node)

if __name__ == '__main__':
    main()
