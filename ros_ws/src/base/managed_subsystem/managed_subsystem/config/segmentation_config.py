from python_base_class.node_config import CommunicationTypes
from sensor_msgs.msg import Image
from system_interfaces.msg import SensorFusion

comm_types = [
    {
        "comm_type": CommunicationTypes.PUBLISHER,
        "name": "/segmentation",
        "msg_type": Image,
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/sensors_fused",
        "msg_type": SensorFusion,
        "callback": "fusion_callback",
        "always_on": False,
    },
]
