import os
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time
import numpy as np
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message

class ImageToRosbag(Node):
    def __init__(self, data_folder, output_bag):
        super().__init__('image_to_rosbag')
        self.bridge = CvBridge()
        self.data_folder = data_folder
        self.output_bag = output_bag

    def create_rosbag_with_images(self):
        # Initialize the writer
        storage_options = StorageOptions(uri=self.output_bag, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        writer = SequentialWriter()
        writer.open(storage_options, converter_options)

        # Create a topic for rgb
        topic_info = TopicMetadata(
            name='/rgb_raw',
            type='sensor_msgs/msg/Image',
            serialization_format='cdr')
        writer.create_topic(topic_info)
        
        # Create a topic for depth
        topic_info = TopicMetadata(
            name='/depth_raw',
            type='sensor_msgs/msg/Image',
            serialization_format='cdr')
        writer.create_topic(topic_info)
        
        # Create a topic for ground truth segmentation
        topic_info = TopicMetadata(
            name='/gt_segmentation',
            type='sensor_msgs/msg/Image',
            serialization_format='cdr')
        writer.create_topic(topic_info)
       

        # Get list of image files in the data folder
        rgb_files = sorted([f for f in os.listdir(os.path.join(self.data_folder, "rgb")) if f.endswith(('.png', '.jpg', '.jpeg'))])
        depth_files = sorted([f for f in os.listdir(os.path.join(self.data_folder, "depth")) if f.endswith(('.png', '.jpg', '.jpeg'))])
        gt_files = sorted([f for f in os.listdir(os.path.join(self.data_folder, "semantic")) if f.endswith(('.png', '.jpg', '.jpeg'))])

        idx = 0
        for rgb_file, depth_file, gt_file in zip(rgb_files, depth_files, gt_files):
            # Common timestamp
            timestamp = rclpy.time.Time(seconds=idx/10).to_msg()

            # RGB
            image_path = os.path.join(self.data_folder, "rgb", rgb_file)
            cv_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            ros_image.header.stamp = timestamp
            writer.write('/rgb_raw', serialize_message(ros_image), int(timestamp.sec*10**9) + timestamp.nanosec)

            # Depth
            image_path = os.path.join(self.data_folder, "depth", depth_file)
            cv_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED) #.astype(np.uint16)
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="16UC1")
            ros_image.header.stamp = timestamp
            writer.write('/depth_raw', serialize_message(ros_image), int(timestamp.sec*10**9) + timestamp.nanosec)

            # Ground truth
            image_path = os.path.join(self.data_folder, "semantic", gt_file)
            cv_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="8UC1")
            ros_image.header.stamp = timestamp
            writer.write('/gt_segmentation', serialize_message(ros_image), int(timestamp.sec*10**9) + timestamp.nanosec)

            self.get_logger().info(f"Added {rgb_file} to {self.output_bag} with timestamp {timestamp.sec}.{timestamp.nanosec}")

            idx += 1

def main(args=None):
    rclpy.init(args=args)

    data_folder = 'data'  # Replace with the path to your image folder
    output_bag = 'output_bag'  # Replace with the desired output bag file name

    image_to_rosbag = ImageToRosbag(data_folder, output_bag)
    image_to_rosbag.create_rosbag_with_images()

    image_to_rosbag.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
