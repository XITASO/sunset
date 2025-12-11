import rclpy
from typing import List
import rclpy.parameter
import os
import datetime
from system_interfaces.msg import ExperimentLogging

from python_base_class.engel_base_class import ENGELBaseClass
from experiment_setup.config.experiment_logger_config import comm_types

from ament_index_python.packages import get_package_share_directory


class ExperimentLoggerNode(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List[dict],
        node_name: str = "experiment_logger",
        param_file: str = "params.yaml",
    ) -> None:
        """
        Initializes the ExperimentLoggerNode.

        Parameters:
        comm_types (List[dict]): A list containing communication types for the node.
        node_name (str): The name of the node. Default is "experiment_logger_node".
        param_file (str): The name of the parameter file. Default is "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("experiment_setup"), "resources", param_file
        )

        self.csv_write_time_period = None
        self.log_file_name = None
        self.log_file_path = None
        self.new_log_file_path = None

        super().__init__(node_name, comm_types, config_file)

        self.trigger_configure()
        self.trigger_activate()

        # Define data for logging
        self.columns = [
            "timestamp",
            "scenario",
            "source",
            "rule_name",
            "strategy_name",
            "strategy_hash",
            "strategy_status",
            "adaptation_type",
            "adaptation_status",
            "success",
            "iou",
            "vehicle_iou",
            "road_iou",
            "gt_failure_name",
            "is_gt_failure",
        ]
        self.logged_data = [",".join(self.columns)]
        self.log_dir = os.path.join("/home/dockuser/ros_ws/log_dump")
        
        os.makedirs(self.log_dir, exist_ok=True)
        self.timer = self.create_timer(self.csv_write_time_period, self._write_to_csv)
        

    def logger_callback(self, log_msg: ExperimentLogging) -> None:
        """
        Log the incoming message to the logged_data.

        Parameters:
        log_msg (ExperimentLogging): A logging message to be written to the log file.
        """
        new_data = []
        current_scenario = str(getattr(log_msg, "scenario", "") or "").strip()
        self.logger.info(f"Logger callback: {self.log_file_path}")

        create_or_rotate = False
        if current_scenario:
            if self.log_file_name is None:
                create_or_rotate = True
            else:
                if current_scenario not in self.log_file_name:
                    create_or_rotate = True

        if create_or_rotate:
            timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            if self.log_file_name is None:
                # First file
                self.log_file_name = f"{timestamp}{current_scenario}.csv"
                self.log_file_path = os.path.join(self.log_dir, self.log_file_name)
                self.logger.info(f"Created log file: {self.log_file_path}")
            else:
                # Rotate file name by appending scenario token
                base, ext = os.path.splitext(self.log_file_name)
                new_name = f"{base}{current_scenario}{ext}"
                self.new_log_file_path = os.path.join(self.log_dir, new_name)

        for attr in self.columns:
            value = getattr(log_msg, attr)
            new_data.append(str(value))
        self.logged_data.append(",".join(new_data))

    def _write_to_csv(self) -> None:
        """Write the current logs to a csv file."""

        self.logger.info(f"Wrote file to {self.log_file_path}")

        if self.log_file_path is None:
            return

        if self.new_log_file_path is None:
            with open(self.log_file_path, "w") as f:
                f.write("\n".join(self.logged_data))
        else:
            old_data = []
            with open(self.log_file_path, "r") as f:
                old_data = f.readlines()
            with open(self.new_log_file_path, "w") as f:
                f.writelines(old_data)
                f.write("\n".join(self.logged_data))
            os.remove(self.log_file_path)
            self.log_file_path = self.new_log_file_path
            self.new_log_file_path = None


def main() -> None:
    """
    Main function to initialize and run the ExperimentLoggerNode.
    """
    rclpy.init()
    test = ExperimentLoggerNode(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
