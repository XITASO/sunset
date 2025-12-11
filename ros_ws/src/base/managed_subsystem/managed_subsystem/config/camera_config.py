from python_base_class.node_config import CommunicationTypes
from sensor_msgs.msg import Image
from system_interfaces.srv import SetHeartbeatStatus
from system_interfaces.msg import IsImageDegraded

comm_types = [
    {
        "comm_type": CommunicationTypes.PUBLISHER,
        "name": "/rgb_camera",
        "msg_type": Image,
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/rgb_raw",
        "msg_type": Image,
        "callback": "raw_rgb_callback",
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.PUBLISHER,
        "name": "/is_image_degraded",
        "msg_type": IsImageDegraded,
        "always_on": False,
    },
    # Optional service for setting degradation in scenarios
    {
        "comm_type": CommunicationTypes.SERVICE,
        "name": "/degrade_camera_heartbeat",
        "srv_type": SetHeartbeatStatus,
        "callback": "set_heartbeat_service",
        "always_on": False,
    },
]
