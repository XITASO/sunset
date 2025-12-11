from python_base_class.node_config import CommunicationTypes
from sensor_msgs.msg import Image
from system_interfaces.msg import SegmentationMetrics, ExperimentLogging

comm_types = [
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/gt_segmentation",
        "msg_type": Image,
        "callback": "gt_callback",
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/segmentation",
        "msg_type": Image,
        "callback": "pred_callback",
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.PUBLISHER,
        "name": "/seg_metrics",
        "msg_type": SegmentationMetrics,
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.PUBLISHER,
        "name": "/evaluator_log",
        "msg_type": ExperimentLogging,
        "always_on": False,
    },
]
