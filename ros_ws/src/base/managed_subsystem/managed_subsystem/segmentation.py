import rclpy
from typing import List
from rclpy.lifecycle import State, TransitionCallbackReturn
from system_interfaces.msg import SensorFusion, FusionType
from cv_bridge import CvBridge
import os

from python_base_class.node_config import CommunicationTypes
from python_base_class.engel_base_class import ENGELBaseClass
from managed_subsystem.config.segmentation_config import comm_types
from managed_subsystem.utils.segmentation_logic import UAVidSegmenter

from ament_index_python.packages import get_package_share_directory
from rclpy.lifecycle import LifecycleState
import time


class SegmentationNode(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List,
        node_name: str = "segmentation",
        param_file: str = "params.yaml",
    ) -> None:
        """
        Initializes the SegmentationNode.

        Parameters:
        comm_types (List): A list of communication configurations.
        node_name (str): The name of the ROS node. Defaults to "segmentation_node".
        param_file (str): The configuration file containing parameters, located in the package resources. Defaults to "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("managed_subsystem"), "resources", param_file
        )
        self.cuda_out_of_memory = None # should resolve after restart
        self.gpu_failure = None # should resolve after redeploy
        self.lifecycle_activation_delay = None
        super().__init__(node_name, comm_types, config_file, namespace="managed_subsystem")

        self.segmenter = UAVidSegmenter(checkpoints_path=".data/checkpoints", logger=self.logger)
        self.bridge = CvBridge()
        self.entropy_values = list()
        self._activation_timer = None  # Timer for delayed activation
        self._activation_delay_s = 0.0

        self.map_modality2string = {
            0: "fusion",
            1: "rgb",
            2: "depth",
        }
        
        # Trigger lifecycle transitions after initialization
        self.trigger_configure()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node - called before activation."""
        # Schedule activation with a delay specified in parameters
        self._activation_delay_s = float(
            self.lifecycle_activation_delay if self.lifecycle_activation_delay is not None else 5.0
        )

        if self._activation_timer is not None:
            self._activation_timer.cancel()
            self._activation_timer = None

        now_ns = self.get_clock().now().nanoseconds
        if now_ns == 0:
            self._activation_timer = self.create_timer(0.1, self._wait_for_valid_clock_and_arm_activation)
            self.get_logger().info(
                f"Node configured, waiting for first sim clock tick before applying activation delay of {self._activation_delay_s} seconds"
            )
        else:
            self._activation_timer = self.create_timer(self._activation_delay_s, self._trigger_activation)
            self.get_logger().info(
                f"Node configured, there is a simulated delay of {self._activation_delay_s} seconds"
            )
        self.invoke_parameter_change_callback(
            {
                "gpu_failure": False,
            }
        )
        return super().on_configure(state)

    def _wait_for_valid_clock_and_arm_activation(self):
        """Wait until ROS time starts, then arm the real activation delay timer."""
        if self.get_clock().now().nanoseconds == 0:
            return

        if self._activation_timer is not None:
            self._activation_timer.cancel()
            self._activation_timer = None

        self._activation_timer = self.create_timer(self._activation_delay_s, self._trigger_activation)
        self.get_logger().info(
            f"First sim clock tick received, activation will be triggered after {self._activation_delay_s} seconds"
        )

    def _trigger_activation(self):
        """Callback to trigger activation after delay."""
        if self._activation_timer is not None:
            self._activation_timer.cancel()
            self._activation_timer = None
        # Trigger the activation transition
        self.trigger_activate()

    def on_activate(self, state: LifecycleState):
        self.invoke_parameter_change_callback(
            {
                "cuda_out_of_memory": False,
            }
        )
        return super().on_activate(state)

    def fusion_callback(self, fusion_data: SensorFusion) -> None:
        """
        Callback function for processing sensor fusion data.

        Parameters:
        fusion_data (SensorFusion): The ROS message containing sensor fusion data.
        """
        if self.cuda_out_of_memory or self.gpu_failure:
            return
        # In try statement, as image encoding is sometimes not recognized on first image during startup. Will skip these.
        try:
            depth_image, rgb_image = None, None
            if (
                fusion_data.modality.fusion_type == FusionType.DEPTH
                or fusion_data.modality.fusion_type == FusionType.FUSION
            ):
                depth_image = self.bridge.imgmsg_to_cv2(fusion_data.depth, desired_encoding="16UC1")
            if (
                fusion_data.modality.fusion_type == FusionType.RGB
                or fusion_data.modality.fusion_type == FusionType.FUSION
            ):
                rgb_image = self.bridge.imgmsg_to_cv2(fusion_data.rgb, desired_encoding="bgr8")
            data = {"rgb": rgb_image, "depth": depth_image}
        except Exception as e:
            self.logger.info(f"Error processing fusion data: {e}")
            return

        segmentation, entropy = self.segmenter.inference(
            data, modality=self.map_modality2string[fusion_data.modality.fusion_type]
        )
        self.add_diagnostic_value(key="entropy", value=str(entropy))
        #self.entropy_values.append(entropy)

        seg_msg = self.bridge.cv2_to_imgmsg(segmentation, encoding="8UC1")
        seg_msg.header = fusion_data.header
        seg_msg.header.frame_id = f"{self.ns}/segmentation"

        publisher = self.get_comm_object("/segmentation", comm_type=CommunicationTypes.PUBLISHER)
        publisher.publish(seg_msg)


def main() -> None:
    """
    The main function to initialize and spin the SegmentationNode.
    """
    rclpy.init()
    test = SegmentationNode(comm_types=comm_types)
    
    rclpy.spin(test)
    #except:

    #    xs = [x for x in range(len(test.entropy_values))]
    #    plt.plot(xs, test.entropy_values)
    #    plt.savefig('entropy.pdf')
    #    # Make sure to close the plt object once done
    #    plt.close()



if __name__ == "__main__":
    main()
