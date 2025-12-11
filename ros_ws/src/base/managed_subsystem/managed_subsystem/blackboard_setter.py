import rclpy
from rcl_interfaces.msg import ParameterValue, Parameter, ParameterType
from typing import List, Any

from python_base_class.engel_base_class import ENGELBaseClass
from managed_subsystem.config.blackboard_config import comm_types
from ament_index_python.packages import get_package_share_directory
from system_interfaces.msg import Heartbeat, IsImageDegraded, SetBlackboardGroup, Flightphase, SetBlackboard
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray

import numpy as np
import os

import os
from ament_index_python.packages import get_package_share_directory


class BlackboardSetter(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List,
        node_name: str = "blackboard_setter",
        param_file: str = "params.yaml",
    ):
        config_file = os.path.join(
            get_package_share_directory("managed_subsystem"), "resources", param_file
        )
        super().__init__(node_name, comm_types, config_file, namespace="managed_subsystem")

        self.trigger_configure()
        self.trigger_activate()
        self.data = dict()
        self.global_node = String(data='global')

    def create_parameter(self, name: str, value: Any) -> Parameter:
        """
        Creates a ROS2 Parameter message from a given name and value.

        Args:
            name (str): The name of the parameter.
            value (Any): The value of the parameter. Supported types are int, float, bool, and str.

        Returns:
            Parameter: A populated ROS parameter message.
        
        Raises:
            ValueError: If the type of 'value' is not supported.
        """
        parameter = Parameter()
        parameter.name = name
        parameter_value = ParameterValue()
        parameter_value.type = ParameterType.PARAMETER_NOT_SET        

        # Watch out, some instances are sub types of others, so the order is important
        if isinstance(value, int):
            parameter_value.type = ParameterType.PARAMETER_INTEGER
            parameter_value.integer_value = value
        if isinstance(value, float):
            parameter_value.type = ParameterType.PARAMETER_DOUBLE
            parameter_value.double_value = value
        if isinstance(value, bool):
            parameter_value.type = ParameterType.PARAMETER_BOOL
            parameter_value.bool_value = value
        if isinstance(value, str):
            parameter_value.type = ParameterType.PARAMETER_STRING
            parameter_value.string_value = value
        if parameter_value.type == ParameterType.PARAMETER_NOT_SET:
            raise ValueError(f"Unsupported parameter type for value: {value}")

        parameter.value = parameter_value
        return parameter

    def get_node_from_topic(self, topic_name: str) -> String:
        namespace = self.get_publishers_info_by_topic(topic_name)[0].node_namespace
        name = self.get_publishers_info_by_topic(topic_name)[0].node_name
        return String(data=f"{namespace}/{name}")

    def publish_flight_phase(self, topic_name: str = "/flight_phase") -> None:
        """Adds the current flightphase to the blackboard messages."""
        if not topic_name in self.data:
            return
        value = self.data[topic_name]
        if value['current_value'] is None:
            return

        msg = SetBlackboard()
        msg.node = self.global_node
        
        msg.blackboard_value = self.create_parameter(
            "current_phase", value['current_value']
        )
        self.bundle_msg.bb_params.append(msg)

    def calculate_frequency(self, timestamps: List) -> float:
        """
        Calculates the frequency of events from a list of timestamps.

        Args:
            timestamps (List): List of timestamps in nanoseconds.

        Returns:
            float: Average frequency in Hertz.
        """
        # Calculate individual frequency of each message in buffer as count/timespan.
        # Set frequency as highest individual frequency
        timestamps_array = np.array(timestamps[::-1])
        num_msgs = timestamps_array.shape[0]
        now_ns = self.get_clock().now().nanoseconds
        diff = now_ns - timestamps_array
        freqs = np.arange(1, num_msgs+1) / diff * 1e9
        freq = np.max(freqs)
        return freq

    def publish_frequencies(self, topic_name: str, value: List, buffer_length: int = 8):
        """
        Adds the calculated frequency of messages on a given topic to the published messages.

        Args:
            topic_name (str): Name of the topic.
            value (List): Dictionary containing 'timestamps' and other values 
                          relevant to the topic.
            buffer_length (int): How many values to keep in the timestamp dictionary.
        """
        # Calculate the frequencies of the messages for this topic
        value['timestamps'] = value['timestamps'][-buffer_length:]
        freq = self.calculate_frequency(value['timestamps'])
        msg = SetBlackboard()
        msg.node = self.get_node_from_topic(topic_name)
        msg.blackboard_value = self.create_parameter(
            f"{topic_name.split('/')[-1]}_freq", freq
        )

        self.bundle_msg.bb_params.append(msg)

    def publish_heartbeat_value(self, topic_name: str, value: dict):
        """
        Adds heartbeat status if there is a change in the current value to the published messages.

        Args:
            topic_name (str): Name of the topic.
            value (dict): Dictionary containing current and last published values.
        """
        if value['current_value'] is None:
            return
        if not "heartbeat" in topic_name:
            return

        # Publish the heartbeat status
        msg = SetBlackboard()
        msg.node = self.get_node_from_topic(topic_name)
        msg.blackboard_value = self.create_parameter(
                    f"{topic_name.split('/')[-1]}_status", value['current_value']
                )
        value.update({'last_published_value': value['current_value']})
        # self.get_logger().info(f"Publish heartbeat status for {topic_name}")
        self.bundle_msg.bb_params.append(msg)

        # Publish the current lifecycle state
        msg.blackboard_value = self.create_parameter(
                    f"{topic_name.split('/')[-1].replace('heartbeat', 'lc_state')}", value['lc_state']
                )
        # self.get_logger().info(f"Publish heartbeat status for {topic_name}")
        self.bundle_msg.bb_params.append(msg)


    def publish_image_degradation(self, topic_name: str = "/is_image_degraded", patience: int = 3):
        """
        Adds image degradation status if there is a change in the current value to the published messages.

        Args:
            topic_name (str): Name of the image degradtation topic.
            patience (int): How many timesteps the change needs to be consistent to publish change.
        """
        if not topic_name in self.data:
            return
        value = self.data[topic_name]
        if value['current_value'] is None:
            return
        if value['current_value'] == value['last_published_value']:
            value["count_since_change"] = 0
            return
        if value["count_since_change"] < patience:
            value["count_since_change"] += 1
            return
        
        msg = SetBlackboard()
        msg.node = self.global_node
        msg.blackboard_value = self.create_parameter(
                    "image_degraded", value['current_value']
                )
        value['last_published_value'] = value['current_value']
        self.bundle_msg.bb_params.append(msg)

    def publish_diagnostics(self) -> None:
        
        for diagnostic in ['water_visibility', 'thruster_failure', 'battery_level']:
            if diagnostic in self.data:
                msg = SetBlackboard()
                msg.node = self.global_node
                msg.blackboard_value = self.create_parameter(diagnostic, self.data[diagnostic])
                self.bundle_msg.bb_params.append(msg)

    def publisher_callback(self) -> None:
        """
        Callback function for checking and publishing frequency and heartbeat status 
        for all topics stored in the data dictionary.
        """
        self.bundle_msg = SetBlackboardGroup()
        #for topic_name, value in self.data.items():
        #    if len(value['timestamps']) > 3:
        #        self.publish_frequencies(topic_name, value)
        #    self.publish_heartbeat_value(topic_name, value)
        #self.publish_image_degradation()
        #self.publish_flight_phase()
        self.publish_diagnostics()

        # bundel all outgoing messages into one
        publisher = self.get_comm_object("/blackboard_group")
        publisher.publish(self.bundle_msg)
            
    def subscription_callback(self, msg: Any):
        """
        Callback function for handling received messages. Updates the data dictionary 
        with received message timestamps and values.

        Args:
            msg (Any): Received message, supporting messages with a header that includes 
                       a frame_id and is an instance of Heartbeat, IsImageDegraded, or Flightphase.
        """
        if msg.header.frame_id not in self.data:
            self.data[msg.header.frame_id] = {
                'timestamps': list(),
                'current_value': None,
                'lc_state': None,
                'last_published_value': None,
                'count_since_change': 0,
            }

        self.data[msg.header.frame_id]['timestamps'].append(
            self.get_clock().now().nanoseconds
        )

        if isinstance(msg, Heartbeat):
            self.data[msg.header.frame_id]['current_value'] = msg.status
            self.data[msg.header.frame_id]['lc_state'] = msg.lc_state
        
        if isinstance(msg, IsImageDegraded):
            self.data[msg.header.frame_id]['current_value'] = msg.is_degraded

        if isinstance(msg, Flightphase):
            self.data[msg.header.frame_id]['current_value'] = msg.flightphase

    def diagnostics_callback(self, msg: DiagnosticArray):
        diagnostic_msg = msg.status[0]
        if 'water_visibility' in diagnostic_msg.name:
            self.data['water_visibility'] = float(diagnostic_msg.values[0].value)
        elif 'battery' in diagnostic_msg.name:
            self.data['battery_level'] = float(diagnostic_msg.values[0].value)
        elif 'thruster' in diagnostic_msg.name:
            self.data['thruster_failure'] = True if diagnostic_msg.values[0].value == 'ERROR' or diagnostic_msg.values[0].value == 'FAILURE' else False
        else:
            # Nothing to do here since we only monitor these three things in the diagnostics topic
            return


def main():
    rclpy.init()
    test = BlackboardSetter(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
