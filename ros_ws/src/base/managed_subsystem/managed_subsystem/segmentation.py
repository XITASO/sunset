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
        self.do_drop_segmentation = None # should resolve after restart
        self.do_hard_drop_segmentation = None # should resolve after redeploy
        super().__init__(node_name, comm_types, config_file, namespace="managed_subsystem")

        self.trigger_configure()
        self.trigger_activate()

        self.segmenter = UAVidSegmenter(checkpoints_path=".data/checkpoints", logger=self.logger)
        self.bridge = CvBridge()
        self.entropy_values = list()

        self.map_modality2string = {
            0: "fusion",
            1: "rgb",
            2: "depth",
        }

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.invoke_parameter_change_callback(
            {
                "do_hard_drop_segmentation": False,
            }
        )
        time.sleep(1)
        return super().on_configure(state)

    def on_activate(self, state: LifecycleState):
        self.invoke_parameter_change_callback(
            {
                "do_drop_segmentation": False,
            }
        )
        return super().on_activate(state)

    def fusion_callback(self, fusion_data: SensorFusion) -> None:
        """
        Callback function for processing sensor fusion data.

        Parameters:
        fusion_data (SensorFusion): The ROS message containing sensor fusion data.
        """
        if self.do_drop_segmentation or self.do_hard_drop_segmentation:
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
