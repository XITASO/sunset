import rclpy
from typing import List
import os
import yaml
from system_interfaces.msg import ExperimentLogging
import subprocess
import re

from python_base_class.engel_base_class import ENGELBaseClass
from rcl_interfaces.srv import SetParameters
from experiment_setup.config.scenario_executor_config import comm_types
from experiment_setup.failure_selector import CriticalitySelector

from ament_index_python.packages import get_package_share_directory


class ScenarioExecutor(ENGELBaseClass):
    """This node executes a scenario from a given scenario.yaml file.
    Make sure to run it with use_sim_time=True to sync with the rosbag time.
    ros2 run experiment_setup scenario_executor --ros-args --param use_sim_time:=true"""

    def __init__(
        self,
        comm_types: List[dict],
        node_name: str = "scenario_executor",
        param_file: str = "params.yaml",
        # scenario_file: str = "seams_eval_example.yaml",
    ) -> None:
        """
        Initializes the ScenarioExecutor node.

        Parameters:
        comm_types (List[dict]): A list containing communication types for the node.
        node_name (str): The name of the node. Default is "message_drop_node".
        param_file (str): The name of the parameter file. Default is "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("experiment_setup"), "resources", param_file
        )
        super().__init__(node_name, comm_types, config_file, namespace="experiment_setup")

        self.trigger_configure()
        self.trigger_activate()

        log_dump_dir = os.path.join("/home/dockuser/ros_ws/log_dump")

        self.scenario_file = None
        directory = os.path.join(get_package_share_directory('experiment_setup'), 'resources/scenarios/fixedScenarios')
        for fixed_scenario_file in sorted(os.listdir(directory)):
            identifier_in_file = self._get_scenario_identifier(fixed_scenario_file)
            self.logger.info(f"Checking fixed scenario file {fixed_scenario_file} with identifier {identifier_in_file}")
            found_file = False
            for log_file in os.listdir(log_dump_dir):
                if identifier_in_file in log_file:
                    found_file = True
                    break
            if not found_file:
                self.scenario_file = fixed_scenario_file
                self.logger.info(f"Scenario not yet executed. Executing: {self.scenario_file}")
                break
                
        
        # if scenario_file == "":
        #     scenario_file = self.scenario_file # type: ignore
        #     scenario_file = "scenario__001.yaml"
        #     self.get_logger().info(f"Using scenario file from params definition: {scenario_file}")

        # Add a one-shot timer for the first callback after 1 second
        self.first_timer = self.create_timer(2.0, self._first_clock_callback)

        # Placeholder for the repeating timer
        self.repeating_timer = None

        # Read in all scenario specific events
        self.scenario_file_path = os.path.join(
            get_package_share_directory("experiment_setup"),
            "resources",
            "scenarios",
            "fixedScenarios",
            self.scenario_file
        )
        if os.path.exists(self.scenario_file_path):
            with open(self.scenario_file_path, "rb") as f:
                scenario_data = yaml.load(f, Loader=yaml.FullLoader)
            self.event_selector = CriticalitySelector(scenario_data)
        else:
            self.ordered_events = []

        if not self.validate_parameters():
            self.logger.warn(
                f"Not all parameters are initialized correctly. Every parameter in \
                    the params.yaml file has to be a class member of {self.get_name()}"
            )

    def _first_clock_callback(self):
        """Called once, 1 second after spinning starts."""
        if self.clock_callback() is True:
            # Cancel the one-shot timer
            self.first_timer.cancel()

    def clock_callback(self) -> bool:
        """Triggers new events from the scenario event list according to timestamp."""
        stamp = self.get_clock().now().nanoseconds

        # First, collect all events for each level
        events = {}
        for level in ["Failure", "Warning", "Ok"]:
            try:
                event = self.event_selector.select_element(level)
                events[level] = event
            except ValueError:
                self.logger.warn(f"No event found for criticality level: {level}")
                events[level] = None

        # Check if all required services are present before performing adaptations
        all_services_available = True
        for level, event in events.items():
            if event is None:
                continue
            
            if event.get("type") == "parameter_adaptation":
                service_name = f"{event.get('target_node')}/set_parameters"
                service_client = self.create_client(SetParameters, service_name)
                if service_client is None or not service_client.wait_for_service(timeout_sec=1.0):
                    self.logger.warn(f"Service not available: {service_name}")
                    all_services_available = False

        # Only proceed with adaptations if all services are available
        if not all_services_available:
            self.logger.warn("Not all required services are available. Skipping adaptations.")
            return False

        # Now perform the adaptations
        for level, event in events.items():
            if event is None:
                continue

            # Failure case produced
            publisher = self.get_comm_object("/bt_executor_log")
            log_msg = ExperimentLogging(
                timestamp=stamp,
                scenario=self._get_scenario_identifier(self.scenario_file),
                source="scenario_executor",
                gt_failure_name=event["name"],
            )
            self.logger.warn("Publishing log message for scenario: " + log_msg.scenario)
            publisher.publish(log_msg)

            # Check for the corresponding action for the event type
            if event["type"] == "parameter_adaptation":
                self._adapt_parameters(event)
            else:
                self.logger.warn(f"Unknown event type {event['type']}")
                self.logger.info(f"Time: ")

            # self.scenario_file = self._get_scenario_identifier()
        return True

    def _get_scenario_identifier(self, file_name: str):
        """
        Extract the scenario identifier from the file name.
        E.g., scenario_001.yaml -> _001
        """
        current_file_name = re.match(r"scenario(_\d{3})\.ya?ml", file_name)
        return current_file_name.group(1)
    

    def _adapt_parameters(self, event: dict) -> None:
        """Set a new parameter according to the event.
        
        Parameters:
        event (dict): A dict containing the specific values for an event.
        """
        
        # test if opening new process works better
        process = subprocess.Popen(
            args=[
                "ros2",
                "param",
                "set",
                event["target_node"],
                event["name"],
                str(event["value"]),
            ]
        )

        self.logger.info(
            f"New scenario adaptation: {event['name']} with failure state {event['is_gt_failure']}"
        )

        # Log the expected ground truth of the system state
        publisher = self.get_comm_object("/scenario_executor_log")
        log_msg = ExperimentLogging(
            timestamp=self.get_clock().now().nanoseconds,
            source="/scenario_executor_log",
            gt_failure_name=event["name"],
            is_gt_failure=event["is_gt_failure"],
        )
        publisher.publish(log_msg) # type: ignore


def main() -> None:
    """
    Main function to initialize and run the ScenarioExecutor.
    """
    rclpy.init()

    test = ScenarioExecutor(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
