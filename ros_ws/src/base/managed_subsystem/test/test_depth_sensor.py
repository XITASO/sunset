from python_base_class.engel_test_base_class import EngelClassTestTemplate
import rclpy
from managed_subsystem.depth import DepthNode
from managed_subsystem.config.depth_config import comm_types

def setup_module():
    """Initializes the rclpy library, which should be called once per Python process."""
    rclpy.init(args=None)                                                               

def teardown_module():
    """Shuts down the rclpy library, which should be called once per Python process after all tests have completed."""
    rclpy.shutdown()

class TestDepthSensor(EngelClassTestTemplate):
    def setup_method(self):
        super().setup_method(expected_comm_objects=comm_types, node_to_test=DepthNode)