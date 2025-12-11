from python_base_class.node_config import CommunicationTypes
from sensor_msgs.msg import Image
from system_interfaces.msg import SensorFusion

comm_types = [
    {
        "comm_type": CommunicationTypes.PUBLISHER,
        "name": "/sensors_fused",
        "msg_type": SensorFusion,
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/depth_camera",
        "msg_type": Image,
        "callback": "depth_callback",
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/rgb_camera",
        "msg_type": Image,
        "callback": "rgb_callback",
        "always_on": False,
        "param": "topic_camera_input",
    },
]
