import rclpy
from rclpy.parameter import Parameter
from typing import List
from sensor_msgs.msg import Image
import os
import numpy as np
from cv_bridge import CvBridge
from system_interfaces.msg import ExperimentLogging, Flightphase
from python_base_class.node_config import CommunicationTypes

from python_base_class.engel_base_class import ENGELBaseClass
from managed_subsystem.config.depth_config import comm_types

from ament_index_python.packages import get_package_share_directory
from rclpy.lifecycle import LifecycleState


class DepthNode(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List,
        node_name: str = "depth",
        param_file: str = "params.yaml",
    ) -> None:
        """
        Initializes the DepthNode.

        Parameters:
        comm_types (List): A list of communication configurations.
        node_name (str): The name of the ROS node. Defaults to "depth".
        param_file (str): The configuration file containing parameters, located in the package resources. Defaults to "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("managed_subsystem"), "resources", param_file
        )
        self.depth_degradation = None
        self.do_drop_depth_camera = None
        super().__init__(node_name, comm_types, config_file)


        self.trigger_configure()
        self.trigger_activate()

        self.bridge = CvBridge()

        if not self.validate_parameters():
            self.logger.warn(
                f"Not all parameters are initialized correctly. Every parameter in \
                    the params.yaml file has to be a class member of {self.get_name()}"
            )

    def on_configure(self, state: LifecycleState):
        return super().on_configure(state)

    def depth_callback(self, msg: Image) -> None:
        """
        Callback function for processing depth images.

        Parameters:
        msg (Image): The ROS Image message containing depth data.
        """

        # drop the image
        if self.do_drop_depth_camera:
            # Directly log if message is dropped
            publisher = self.get_comm_object(f"/{self.get_name()}/experiment_log", CommunicationTypes.PUBLISHER)
            log_msg = ExperimentLogging(
                    timestamp=self.get_clock().now().nanoseconds,
                    source=f"/{self.get_name()}/experiment_log",
                    gt_failure_name="depth_image_dropped",
                    is_gt_failure=True,
                )
            publisher.publish(log_msg)
            self.logger.info(f"Dropped msg {msg.header.frame_id} at time {msg.header.stamp.sec+msg.header.stamp.nanosec/1e9:0.3}")
            return
        

        if self.depth_degradation > 0:
            msg = self._degrade_depth(msg)

        publisher = self.get_comm_object("/depth_camera")
        msg.header.frame_id = f"{self.ns}/depth_camera"
        publisher.publish(msg)

    def _degrade_depth(self, msg: Image) -> Image:
        """
        Add noise to the image as a disturbance.

        Parameters:
        msg (Image): The original ROS Image message to be degraded.

        Returns:
        Image: The degraded ROS Image message.
        """
        img_array = self.bridge.imgmsg_to_cv2(msg).astype(float)
        img_array += 16 * self.depth_degradation * (np.random.randn(*img_array.shape) - 0.5)
        msg_new = self.bridge.cv2_to_imgmsg(img_array.astype(np.uint16), encoding=msg.encoding)
        msg_new.header = msg.header
        return msg_new
    
def main() -> None:
    """
    The main function to initialize and spin the DepthNode.
    """
    rclpy.init()
    test = DepthNode(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
