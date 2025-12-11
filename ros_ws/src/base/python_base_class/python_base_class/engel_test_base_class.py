import rclpy
from rclpy.lifecycle import Node as LifeCycleNode
from rclpy.node import Node
import logging
from typing import List
from python_base_class.node_config import CommunicationTypes
from python_base_class.engel_base_class import ENGELBaseClass


def setup_module():
    """Initializes the rclpy library, which should be called once per Python process."""
    rclpy.init(args=None)


def teardown_module():
    """Shuts down the rclpy library, which should be called once per Python process after all tests have completed."""
    rclpy.shutdown()


class EngelClassTestTemplate:
    def setup_method(
        self,
        expected_comm_objects: List,
        node_to_test: LifeCycleNode,
    ):
        assert type(expected_comm_objects) is list
        self.node_to_test: ENGELBaseClass = node_to_test(expected_comm_objects)
        #self.node_to_test.trigger_activate()
        # TODO If we decide to not trigger automatically into the inactive state,
        # we have to add the trigger_configure function here
        self.base_node = Node("base_node")
        self.logger = logging.getLogger(__name__)
        self.expected_objects = {
            CommunicationTypes.PUBLISHER: dict(),
            CommunicationTypes.SUBSCRIPTION: dict(),
            CommunicationTypes.SERVICE: dict(),
            CommunicationTypes.SERVICE_CLIENT: dict(),
        }

        self.get_available_objects = {
            CommunicationTypes.PUBLISHER: self.base_node.get_publisher_names_and_types_by_node,
            CommunicationTypes.SUBSCRIPTION: self.base_node.get_subscriber_names_and_types_by_node,
            CommunicationTypes.SERVICE: self.base_node.get_service_names_and_types_by_node,
            CommunicationTypes.SERVICE_CLIENT: self.base_node.get_client_names_and_types_by_node,
        }
        for object in expected_comm_objects:
            if object["comm_type"] in [
                CommunicationTypes.PUBLISHER,
                CommunicationTypes.SUBSCRIPTION,
            ]:
                exp_obj = object["msg_type"]
                # TODO: Add param specific logic
                # Parameters are not read in at all with the current base test scripts, so correct topic name can also not be accessed.
                if "param" in object:
                    self.logger.warning(f"Could not check the topic {object['name']} since the topic name is a parameter.")
                    continue
            elif object["comm_type"] in [
                CommunicationTypes.SERVICE,
                CommunicationTypes.SERVICE_CLIENT,
            ]:
                exp_obj = object["srv_type"]
            self.expected_objects[object["comm_type"]][object["name"]] = exp_obj

    def parse_ros_class_string(self, class_repr: str) -> str:
        """
        Parses a class str representation that looks like this: <class 'std_msgs.msg._string.String'>
        into a representation like std_msgs/msg/String. The second format will be used when getting information
        about a service/message type via ROS API
        """
        # Extract the class path from the input string
        class_path = class_repr.split("'")[1]

        # Split the path into components
        parts = class_path.split(".")

        # Remove the _<type> part
        parts.pop(2)

        # Join the parts with '/' to create the desired format
        desired_format = "/".join(parts)

        return desired_format

    def test_availability_publishers(self):
        self._test_availability(comm_type=CommunicationTypes.PUBLISHER)

    def test_availability_subscriptions(self):
        self._test_availability(comm_type=CommunicationTypes.SUBSCRIPTION)

    def test_availability_services(self):
        self._test_availability(comm_type=CommunicationTypes.SERVICE)

    def test_availability_service_clients(self):
        self._test_availability(comm_type=CommunicationTypes.SERVICE_CLIENT)

    def _test_availability(self, comm_type: CommunicationTypes):
        """Tests the availability of communication objects of a specified type.

        This method checks if the communication objects (e.g., publishers, subscribers, services, service clients)
        of the specified type are available and match the expected types. It retrieves the available communication
        objects using the `get_available_objects` dictionary and compares them with the expected objects stored
        in the `expected_objects` dictionary.

        Args:
            comm_type (CommunicationTypes): The type of communication object to test (e.g., PUBLISHER, SUBSCRIPTION,
                                            SERVICE, SERVICE_CLIENT).

        Returns:
            None: This method does not return any value. It asserts that all expected communication objects are
                  available and match the expected types.
        """
        available_comm_channels = self.get_available_objects[comm_type](
            self.node_to_test.get_name(), self.node_to_test.get_namespace()
        )
        available_channels_dict = {name: types for name, types in available_comm_channels}

        topic_existing = {
            topic_name: available_channels_dict.get(topic_name)[0]
            == self.parse_ros_class_string(str(topic_type))
            for topic_name, topic_type in self.expected_objects[comm_type].items()
        }
        assert False not in topic_existing

    def test_availability_parameter_class_members(self):
        assert self.node_to_test.validate_parameters()
