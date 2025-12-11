from python_base_class.node_config import CommunicationTypes
from sensor_msgs.msg import Image
from system_interfaces.msg import Flightphase

comm_types = [
    {
        "comm_type": CommunicationTypes.PUBLISHER,
        "name": "/depth_camera",
        "msg_type": Image,
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/depth_raw",
        "msg_type": Image,
        "callback": "depth_callback",
        "always_on": False,
    },
    # Flight phase listener to simulate degradation at high altitudes
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/flight_phase",
        "msg_type": Flightphase,
        "callback": "flight_phase_callback",
        "always_on": False,
    },
]
