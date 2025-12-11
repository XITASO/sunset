from python_base_class.node_config import CommunicationTypes
from sensor_msgs.msg import Image

comm_types = [
    {
        "comm_type": CommunicationTypes.PUBLISHER,
        "name": "/rgb_enhanced",
        "msg_type": Image,
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/rgb_camera",
        "msg_type": Image,
        "callback": "image_callback",
        "always_on": False,
    }
]
