from python_base_class.node_config import CommunicationTypes
from sensor_msgs.msg import Image
from system_interfaces.msg import Heartbeat, SensorFusion, IsImageDegraded, SetBlackboardGroup, Flightphase
from diagnostic_msgs.msg import DiagnosticArray

comm_types = [
    # Outgoing group message
    {
        "comm_type": CommunicationTypes.PUBLISHER,
        "name": "/blackboard_group",
        "msg_type": SetBlackboardGroup,
        "callback": "publisher_callback",
        "time_period": 0.2,
        "always_on": True,
        "q_size": 50
    },
    # All incoming topics to be processed
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/rgb_camera",
        "msg_type": Image,
        "callback": "subscription_callback",
        "always_on": True,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/diagnostics",
        "msg_type": DiagnosticArray,
        "callback": "diagnostics_callback",
        "always_on": True,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/is_image_degraded",
        "msg_type": IsImageDegraded,
        "callback": "subscription_callback",
        "always_on": True,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/camera/heartbeat",
        "msg_type": Heartbeat,
        "callback": "subscription_callback",
        "always_on": True,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/depth_camera",
        "msg_type": Image,
        "callback": "subscription_callback",
        "always_on": True,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/lidar/heartbeat",
        "msg_type": Heartbeat,
        "callback": "subscription_callback",
        "always_on": True,
    },

    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/rgb_enhanced",
        "msg_type": Image,
        "callback": "subscription_callback",
        "always_on": True,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/image_enhancement/heartbeat",
        "msg_type": Heartbeat,
        "callback": "subscription_callback",
        "always_on": True,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/sensors_fused",
        "msg_type": SensorFusion,
        "callback": "subscription_callback",
        "always_on": True,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/sensor_fusion/heartbeat",
        "msg_type": Heartbeat,
        "callback": "subscription_callback",
        "always_on": True,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/segmentation",
        "msg_type": Image,
        "callback": "subscription_callback",
        "always_on": True,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/segmentation/heartbeat",
        "msg_type": Heartbeat,
        "callback": "subscription_callback",
        "always_on": True,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/flight_phase",
        "msg_type": Flightphase,
        "callback": "subscription_callback",
        "always_on": True,
    }
]
