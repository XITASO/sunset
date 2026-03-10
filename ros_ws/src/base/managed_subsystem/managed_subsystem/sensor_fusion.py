import rclpy
from typing import List, Optional, Tuple
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from system_interfaces.msg import SensorFusion
from system_interfaces.msg import FusionType
import os
from rclpy.lifecycle import LifecycleState

from python_base_class.node_config import CommunicationTypes
from python_base_class.engel_base_class import ENGELBaseClass
from managed_subsystem.config.sensor_fusion_config import comm_types

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import numpy as np
import time

class SensorFusionNode(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List,
        node_name: str = "sensor_fusion",
        param_file: str = "params.yaml",
    ) -> None:
        """
        Initializes the SensorFusionNode class.

        Parameters:
        comm_types (List): A list containing the communication types configuration.
        node_name (str): The name of the node. Defaults to "sensor_fusion_node".
        param_file (str): The parameter file path. Defaults to "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("managed_subsystem"), "resources", param_file
        )
        self.topic_camera_input = None 
        self.modality = None
        self.delta_t_threshold = None
        self.image_shift = None
        self.do_recalibration = None
        self.sync_issue = None # should resolve after restart
        self.memory_leakage = None # should resolve after redeploy
        self.lifecycle_activation_delay = None
        super().__init__(node_name, comm_types, config_file, namespace="/managed_subsystem")

        self.data_buffer = {
            "depth": [],  # List of (Image, timestamp) tuples
            "rgb": [],    # List of (Image, timestamp) tuples
        }
        self.max_buffer_size = 15  # Number of images to keep per modality
        self.last_published_timestamps = {"rgb": -1, "depth": -1}  # Track published timestamps to avoid duplicates

        self._activation_timer = None  # Timer for delayed activation
        self._activation_delay_s = 0.0

        if not self.validate_parameters():
            self.logger.warn(
                f"Not all parameters are initialized correctly. Every parameter in \
                    the params.yaml file has to be a class member of {self.get_name()}"
            )
        
        # Trigger lifecycle transitions after initialization
        self.trigger_configure()

    def on_configure(self, state: LifecycleState):
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
                "memory_leakage": False,
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
                "sync_issue": False,
            }
        )
        return super().on_activate(state)

    def rgb_callback(self, image: Image) -> None:
        """
        Callback function for RGB image data.

        Parameters:
        image (Image): The RGB image message.
        """
        self.add_to_data_buffer(image, "rgb")

    def depth_callback(self, image: Image) -> None:
        """
        Callback function for depth image data.

        Parameters:
        image (Image): The depth image message.
        """
        self.add_to_data_buffer(image, "depth")

    def add_to_data_buffer(self, image: Image, key: str) -> None:
        """
        Adds incoming image data to the data buffer, maintaining up to max_buffer_size entries.

        Parameters:
        image (Image): The image message.
        key (str): The data type key ('rgb' or 'depth').
        """
        time_code = image.header.stamp.sec + image.header.stamp.nanosec / 1e9
        
        # Add new image to buffer
        self.data_buffer[key].append((image, time_code))
        
        # Keep only the most recent max_buffer_size images
        if len(self.data_buffer[key]) > self.max_buffer_size:
            self.data_buffer[key].pop(0)

        self.get_logger().debug(f"Received image from modality {key} with time {time_code}. Buffer size: {len(self.data_buffer[key])}")
        
        if self.modality == FusionType.FUSION:
            best_match = self.find_closest_match()
            if best_match is not None:
                self.publish_fused_data(best_match)
        else:
            self.publish_fused_data()

    def find_closest_match(self) -> Optional[Tuple]:
        """
        Finds the closest matching RGB and depth image pair based on timestamps.
        Only returns a match if it hasn't been published before.

        Returns:
        tuple: A tuple of ((rgb_image, rgb_time), (depth_image, depth_time)) or None if no valid match found.
        """
        if not self.data_buffer["rgb"] or not self.data_buffer["depth"]:
            return None
        
        best_match = None
        min_delta = float('inf')
        
        # Find the pair with minimum timestamp difference
        for rgb_img, rgb_time in self.data_buffer["rgb"]:
            for depth_img, depth_time in self.data_buffer["depth"]:
                delta = abs(rgb_time - depth_time)
                
                # Check if this pair is within threshold and has minimum delta
                if delta <= self.delta_t_threshold and delta < min_delta:
                    # Avoid publishing the same pair twice
                    if not (rgb_time == self.last_published_timestamps["rgb"] and 
                            depth_time == self.last_published_timestamps["depth"]):
                        min_delta = delta
                        best_match = ((rgb_img, rgb_time), (depth_img, depth_time))
        
        if best_match is not None:
            rgb_pair, depth_pair = best_match
            self.get_logger().debug(
                f"Found closest match with delta_t = {min_delta:.6f}s (RGB: {rgb_pair[1]:.3f}, Depth: {depth_pair[1]:.3f})"
            )
            # Update last published timestamps
            self.last_published_timestamps["rgb"] = rgb_pair[1]
            self.last_published_timestamps["depth"] = depth_pair[1]
        
        return best_match
    

    def shift_image_right(self, cv_image: np.ndarray) -> np.ndarray:
        # Get image dimensions
        height, width = cv_image.shape[:2]

        # Create a new array filled with zeros
        shifted_image = np.zeros_like(cv_image)

        # Shift the image
        if self.image_shift < width: # type: ignore
            shifted_image[:, self.image_shift:] = cv_image[:, :-self.image_shift] # type: ignore

        return shifted_image

    def process_ros_image_message(self, ros_image: Image) -> Image:
        # Convert ROS Image to CV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')

        # Shift the image to the right
        shifted_cv_image = self.shift_image_right(cv_image)

        # Convert back to ROS Image message
        ros_shifted_image = bridge.cv2_to_imgmsg(shifted_cv_image, encoding="bgr8")

        return ros_shifted_image

    def publish_fused_data(self, best_match: Optional[Tuple] = None) -> None:
        """
        Publishes the fused sensor data.
        
        Parameters:
        best_match (Optional[Tuple]): A tuple of ((rgb_image, rgb_time), (depth_image, depth_time)) 
                                     if using fusion mode with closest match. None for non-fusion modes.
        """
        if self.sync_issue or self.memory_leakage:
            return

        if self.do_recalibration:
            self.logger.info("Sensor fusion is doing recalibration")
            self.invoke_parameter_change_callback(
                {
                    "image_shift": 0,
                    "do_recalibration": False
                }
            )
        
        header = Header()
        header.frame_id = f"{self.ns}/sensors_fused"
        
        # Initialize with default empty images
        rgb_to_publish = Image()
        depth_to_publish = Image()
        
        # For fusion mode with closest match
        if best_match is not None:
            (rgb_img, rgb_time), (depth_img, depth_time) = best_match
            header.stamp = rgb_img.header.stamp
            rgb_to_publish = rgb_img if self.image_shift == 0 else self.process_ros_image_message(rgb_img)
            depth_to_publish = depth_img
        # For single modality or fallback
        else:
            if self.modality == FusionType.DEPTH and self.data_buffer["depth"]:
                header.stamp = self.data_buffer["depth"][-1][0].header.stamp
                depth_to_publish = self.data_buffer["depth"][-1][0]
            elif self.data_buffer["rgb"]:
                header.stamp = self.data_buffer["rgb"][-1][0].header.stamp
                rgb_to_publish = self.data_buffer["rgb"][-1][0]
                if self.image_shift != 0:
                    rgb_to_publish = self.process_ros_image_message(rgb_to_publish)
            else:
                return  # No data to publish

        out_msg = SensorFusion(
            depth=depth_to_publish,
            rgb=rgb_to_publish,
            modality=FusionType(fusion_type=self.modality),
            header=header,
        )

        publisher = self.get_comm_object("/sensors_fused", comm_type=CommunicationTypes.PUBLISHER)
        publisher.publish(out_msg) # type: ignore


def main() -> None:
    """
    Main function to initiate the SensorFusionNode.
    """
    rclpy.init()
    test = SensorFusionNode(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
