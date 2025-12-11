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

data_folder = ".data"

# Get list of image files in the data folder
rgb_files = sorted(
    [
        f
        for f in os.listdir(os.path.join(data_folder, "rgb"))
        if f.endswith((".png", ".jpg", ".jpeg"))
    ]
)

idx = 0
means, stds = [], []
for rgb_file in rgb_files:
    image_path = os.path.join(data_folder, "rgb", rgb_file)
    cv_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    mean = np.mean(cv_image, axis=(0, 1))
    std = np.std(cv_image, axis=(0, 1))

    means.append(mean)
    stds.append(std)

means = np.array(means)
stds = np.array(stds)

mean_avg = np.mean(means, axis=0)
std_avg = np.mean(stds, axis=0)

print(mean_avg)
print(std_avg)
