import os
import rclpy
from rclpy.lifecycle import LifecycleState, Node
from ros2topic.api import get_topic_names_and_types
from rcl_interfaces.msg import ParameterValue, Parameter, ParameterType

from system_interfaces.msg import SetBlackboardGroup
import json
from importlib import import_module
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
import numpy as np


class AbstractBlackboardSetter(Node):
    def __init__(self, wiring_path: str = ""):
        super().__init__("abstract_bb_setter", namespace="managed_subsystem")
        full_path = os.path.join(
            get_package_share_directory("managed_subsystem"), "resources", wiring_path
        )
        with open(full_path, "r") as file:
            self.wirings = json.load(file)

        self._subs = []
        self._timestamps = {}
        self._rewiring_table = {}
        self._frequencies = {}
        self._diagnostics = {}
        self.WINDOW = 10
        self.timer_renewed = False
        self.valid_freqs = False
        self.lifecycle_activation_delay = None
        self._activation_timer = None  # Timer for delayed activation

        self.create_rewiring_table()

        print("Using the following rewiring:\n", self._rewiring_table)

        self.bundle_msg = SetBlackboardGroup()
        self.bb_pub = self.create_publisher(SetBlackboardGroup, "/blackboard_group", 10)
        self.timer = self.create_timer(0.7, self.timer_callback)

        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray, "/diagnostics", self.diagnostics_callback, 10
        )

        print("\nAll setup: Starting!")
        self.trigger_configure()

    def on_configure(self, state: LifecycleState):
        """Configure the node - called before activation."""
        # Schedule activation with a delay specified in parameters
        delay = self.lifecycle_activation_delay if self.lifecycle_activation_delay is not None else 5.0
        self._activation_timer = self.create_timer(delay, self._trigger_activation)
        self.get_logger().info(f"Node configured, there is a simulated delay of {delay} seconds")
        return super().on_configure(state)

    def _trigger_activation(self):
        """Callback to trigger activation after delay."""
        if self._activation_timer is not None:
            self._activation_timer.cancel()
            self._activation_timer = None
        # subscribers
        for key, value_names in self.wirings.items():
            self._subs.append(self.subscription_factory(key, value_names))

        self.trigger_activate()

    def diagnostics_callback(self, msg: DiagnosticArray):
        for status in msg.status:
            #    if not status.name in self._timestamps:
            #        self._timestamps[status.name] = list()
            #    self._timestamps[status.name].append(Time.from_msg(msg.header.stamp).nanoseconds)
            #    self._frequencies[status.name] = self.calculate_frequency(status.name)
            for v in status.values:
                if "lc_state" in v.key:
                    diagnostic_value = f"{status.name}_{v.key}"
                else:
                    diagnostic_value = f"{status.hardware_id}_{v.key}"
                self._diagnostics[diagnostic_value] = v.value


    def create_rewiring_table(self):
        """
        This function automatically creates a rewiring table to avoid duplicate bb keys
        """
        for key, value_names in self.wirings.items():
            # init table
            if key not in self._rewiring_table:
                self._rewiring_table[key] = {}

            found_duplicates = False
            for value_name in value_names:
                # if we already did this key, continue
                if value_name in self._rewiring_table[key]:
                    continue

                # check all other values for duplicates
                for other_key, other_value_names in self.wirings.items():
                    # skip the current entires
                    if other_key == key:
                        continue
                    if other_key not in self._rewiring_table:
                        # init other key
                        self._rewiring_table[other_key] = {}
                    for other_value_name in other_value_names:
                        if value_name == other_value_name:
                            found_duplicates = True
                            self._rewiring_table[other_key][other_value_name] = (
                                f"{other_key[1:].replace('/', '_')}_" + other_value_name
                            )

                if found_duplicates:
                    self._rewiring_table[key][value_name] = (
                        f"{key[1:].replace('/', '_')}_" + value_name
                    )
                else:
                    # no duplicates? --> just use the raw name
                    self._rewiring_table[key][value_name] = value_name

    def calculate_frequency(self, topic) -> float:
        # rather than max i use a sliding window
        if len(self._timestamps[topic]) >= self.WINDOW:
            self.valid_freqs = True
            timestamps_array = np.array(self._timestamps[topic][-self.WINDOW:][::-1])
            num_msgs = timestamps_array.shape[0]
            now_ns = self.get_clock().now().nanoseconds
            diff = now_ns - timestamps_array
            freqs = np.arange(1, num_msgs + 1) / diff * 1e9
            freq = np.max(freqs)
            # freq = (
            #    len(self._timestamps[topic])
            #    * 1e9
            #    / (self._timestamps[topic][-1] - self._timestamps[topic][0])
            # )
            return freq
        return 0.

    def callback_factory(self, topic, value_names: list):
        """
        creates a callback function to handle the specified topic and values
        """
        self._timestamps[topic] = []

        def callback(data):
            # handle specified values
            for value_name in value_names:
                # rewire
                bb_name = self._rewiring_table[topic][value_name]

                try:
                    value = getattr(data, value_name)
                    bb_msg = self.create_parameter(bb_name, value)
                    self.bundle_msg.bb_params.append(bb_msg)

                except Exception as e:
                    self.get_logger().error(f"Had my problems with {e}")

            # handle frequency
            self._timestamps[topic].append(self.get_clock().now().nanoseconds)
            if len(self._timestamps[topic]) > self.WINDOW:
                self._timestamps[topic] = self._timestamps[topic][-self.WINDOW :]

        return callback

    def subscription_factory(self, topic, value_names):
        """
        identifies the topic info and creates a suitable subscription
        """
        topic_types = None
        self.get_logger().info(f"Searching info for topic: {topic} ...")
        while topic_types is None:
            topic_names_and_types = get_topic_names_and_types(node=self)
            for t_name, t_types in topic_names_and_types:
                if t_name == topic:
                    topic_types = t_types
                    break
        type_str = topic_types[0] if len(topic_types) == 1 else topic_types
        self.get_logger().info(f"Found type {type_str}")

        type_str = type_str.replace("/", ".")
        pkg_name = type_str.rsplit(".", 1)
        msg_class = getattr(import_module(pkg_name[0]), pkg_name[1])

        return self.create_subscription(
            msg_class, topic, self.callback_factory(topic, value_names), 10
        )

    def create_parameter(self, name: str, value) -> Parameter:
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

    def freq_2_param(self):
        """
        creates Parameters out of the calculated frequencies
        """
        for topic, freq in self._frequencies.items():
            start_idx = 1 if topic[0] == "/" else 0
            bb_msg = self.create_parameter(
                f"{topic[start_idx:].replace('/', '_')}_freq", freq
            )
            self.bundle_msg.bb_params.append(bb_msg)

    def diagnostics_2_param(self):
        def determine_parameter_type(value: str):
            if isinstance(value, int):
                return ParameterType.PARAMETER_INTEGER, value
            elif isinstance(value, str):
                # Try to convert the string to an integer
                try:
                    int_value = int(value)
                    return ParameterType.PARAMETER_INTEGER, int_value
                except ValueError:
                    pass

                # Try to convert the string to a float
                try:
                    float_value = float(value)
                    return ParameterType.PARAMETER_DOUBLE, float_value
                except ValueError:
                    pass

                # Try to convert the string to a bool
                try:
                    bool_value = value == "True" or value == "true"
                    return ParameterType.PARAMETER_BOOL, bool_value
                except ValueError:
                    pass

                # Default to string if neither conversion succeeds
                return ParameterType.PARAMETER_STRING, value

        for key, value in self._diagnostics.items():
            param_type, param_value = determine_parameter_type(value)

            parameter_value = ParameterValue()
            if param_type == ParameterType.PARAMETER_INTEGER:
                parameter_value.type = ParameterType.PARAMETER_INTEGER
                parameter_value.integer_value = param_value
            elif param_type == ParameterType.PARAMETER_DOUBLE:
                parameter_value.type = ParameterType.PARAMETER_DOUBLE
                parameter_value.double_value = param_value
            elif param_type == ParameterType.PARAMETER_STRING:
                parameter_value.type = ParameterType.PARAMETER_STRING
                parameter_value.string_value = param_value
            elif param_type == ParameterType.PARAMETER_BOOL:
                parameter_value.type = ParameterType.PARAMETER_BOOL
                parameter_value.bool_value = param_value

            # Create a Parameter object with the key and ParameterValue
            parameter = Parameter(name=key, value=parameter_value)

            self.bundle_msg.bb_params.append(parameter)

    def generate_mock_data(self):
        for name in ["rgb_camera", "sensors_fused", "segmentation"]:
            bb_msg = self.create_parameter(
                f"managed_subsystem_{name}_freq", 0.1
            )
            self.bundle_msg.bb_params.append(bb_msg)

    def check_for_multiple_uncertainties(self) -> bool:
        counter = 0
        for param in self.bundle_msg.bb_params:
            if 'freq' in param.name:
                if param.value.double_value < 1.:
                    counter += 1
            if 'entropy' in param.name:
                if param.value.double_value > 0.06:
                    counter += 1

        if counter >= 2:
            return True
        else:
            return False

    def renew_timer(self):
        if self.timer_renewed == False:
            self.timer_renewed = True
            self.timer.cancel()
            self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        
        for topic in self._timestamps.keys():
           self._frequencies[topic] = self.calculate_frequency(topic)
        if self.valid_freqs:
            self.freq_2_param()
        self.diagnostics_2_param()

        self.bb_pub.publish(self.bundle_msg)
        if self.check_for_multiple_uncertainties():
            self.renew_timer()
        self.bundle_msg.bb_params.clear()

def main():
    rclpy.init()
    test = AbstractBlackboardSetter("bbs_wiring.json")
    rclpy.spin(test)


if __name__ == "__main__":
    main()
