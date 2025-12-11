import rclpy
from typing import List
from sensor_msgs.msg import Image
import os
import numpy as np
from cv_bridge import CvBridge
from system_interfaces.srv import SetHeartbeatStatus
from system_interfaces.msg import ExperimentLogging, IsImageDegraded
from python_base_class.node_config import CommunicationTypes
from builtin_interfaces.msg._time import Time
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn
from rclpy.timer import Timer

from python_base_class.engel_base_class import ENGELBaseClass
from managed_subsystem.config.camera_config import comm_types

from ament_index_python.packages import get_package_share_directory
import time
import cv2


class CameraNode(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List[dict],
        node_name: str = "camera",
        param_file: str = "params.yaml",
    ) -> None:
        """
        Initializes the CameraNode.

        Parameters:
        comm_types (List[dict]): A list containing communication types for the node.
        node_name (str): The name of the node. Default is "camera_node".
        param_file (str): The name of the parameter file. Default is "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("managed_subsystem"), "resources", param_file
        )
        self.image_degradation = None
        self.image_mean_threshold = None
        self.do_drop_rgb_camera = None  # should resolve after restart
        self.do_hard_drop_rgb_camera = None  # should resolve after redeploy
        self.autofocus_needed = None
        self.perform_autofocus = None
        super().__init__(node_name, comm_types, config_file)

        self.trigger_configure()
        self.trigger_activate()

        self.bridge = CvBridge()
        self.reset_timer = None  # Timer for resetting image_degradation

        if not self.validate_parameters():
            self.logger.warn(
                f"Not all parameters are initialized correctly. Every parameter in \
                    the params.yaml file has to be a class member of {self.get_name()}"
            )

    def on_configure(self, state: LifecycleState):
        self.invoke_parameter_change_callback(
            {
                "do_hard_drop_rgb_camera": False,
            }
        )
        time.sleep(1)
        return super().on_configure(state)

    def on_activate(self, state: LifecycleState):
        self.invoke_parameter_change_callback(
            {
                "do_drop_rgb_camera": False,
            }
        )
        return super().on_activate(state)

    def raw_rgb_callback(self, msg: Image) -> None:
        """
        Callback function for processing received raw RGB images.

        This method optionally degrades the image based on the `image_degradation` attribute
        and republishes it on the `/rgb_camera` topic.

        Parameters:
        msg (Image): The incoming RGB image message.
        """
        if self.autofocus_needed:
            time.sleep(0.5)
            self.add_diagnostic_value(key="autofocus_needed", value="True")

        if self.perform_autofocus:
            self.add_diagnostic_value(key="autofocus_needed", value="False")
            self.invoke_parameter_change_callback(
                {
                    "autofocus_needed": False,
                    "perform_autofocus": False
                }
            )

        if self.image_degradation > 0.0 and self.reset_timer is None:  # type: ignore
            # If image_degradation is greater than 0.0 and timer is not already set
            self.start_reset_timer()

        # drop the image
        if self.do_drop_rgb_camera or self.do_hard_drop_rgb_camera:

            # self.logger.info(
            #    f"Dropped msg {msg.header.frame_id} at time {msg.header.stamp.sec+msg.header.stamp.nanosec/1e9:0.3}"
            # )
            # Directly log if message is dropped
            publisher = self.get_comm_object(
                f"/{self.name}/experiment_log", CommunicationTypes.PUBLISHER
            )
            log_msg = ExperimentLogging(
                timestamp=self.get_clock().now().nanoseconds,
                source=f"/{self.name}/experiment_log",
                gt_failure_name="camera_image_dropped",
                is_gt_failure=True,
            )
            # publisher.publish(log_msg)
            return

        # degrade the image
        msg = self._degrade_image(msg)

        # publish the image
        publisher = self.get_comm_object("/rgb_camera")
        msg.header.frame_id = f"{self.ns}/rgb_camera"
        publisher.publish(msg)

    def _degrade_image(self, msg: Image) -> Image:
        """
        Degrades the image by darkening it to simulate dawn conditions.

        Parameters:
        msg (Image): The original image message to be degraded.

        Returns:
        Image: The degraded image message.
        """
        img_array = self.bridge.imgmsg_to_cv2(msg).astype(float)
        if self.autofocus_needed:
            img_array = cv2.GaussianBlur(img_array, (0, 0), 1)
        img_array *= 1 - 0.5 * self.image_degradation

        msg_new = self.bridge.cv2_to_imgmsg(
            img_array.astype(np.uint8), encoding=msg.encoding
        )
        msg_new.header = msg.header
        return msg_new

    def start_reset_timer(self):
        """
        Starts a timer to reset image_degradation to 0.0 after 2 seconds.
        """
        self.reset_timer = self.create_timer(
            20.0,  # Duration: 2 seconds
            self.reset_image_degradation,  # Callback function
        )

    def reset_image_degradation(self):
        """
        Resets image_degradation to 0.0 and cancels the timer.
        """
        self.invoke_parameter_change_callback(
            {
                "image_degradation": 0.0,
            }
        )
        self.reset_timer.cancel()
        self.reset_timer = None
        self.logger.info("Image degradation has been reset to 0.0")

    def set_heartbeat_service(
        self, request: SetHeartbeatStatus.Request, response: SetHeartbeatStatus.Response
    ) -> SetHeartbeatStatus.Response:
        """
        This service is only meant for testing purposes and allows setting the heartbeat status of the camera directly during runtime.
        """
        self.set_heartbeat_status(request.status)
        response.success = True
        self.logger.info(f"Camera hearbeat status was set to {request.status}")

        # Log to experiment log
        publisher = self.get_comm_object(
            f"/{self.get_name()}/experiment_log", CommunicationTypes.PUBLISHER
        )
        log_msg = ExperimentLogging(
            timestamp=self.get_clock().now().nanoseconds,
            source=f"/{self.get_name()}/experiment_log",
            gt_failure_name="camera_heartbeat_degraded",
            is_gt_failure=True,
        )
        publisher.publish(log_msg)

        return response


def main() -> None:
    """
    Main function to initialize and run the CameraNode.
    """
    rclpy.init()
    test = CameraNode(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
