from python_base_class.node_config import CommunicationTypes
from system_interfaces.msg import ExperimentLogging

comm_types = [
    # evaluator logging
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/evaluator_log",
        "msg_type": ExperimentLogging,
        "callback": "logger_callback",
        "always_on": False,
    },
    # camera logging
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/camera/experiment_log",
        "msg_type": ExperimentLogging,
        "callback": "logger_callback",
        "always_on": False,
    },
    # depth logging
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/depth/experiment_log",
        "msg_type": ExperimentLogging,
        "callback": "logger_callback",
        "always_on": False,
    },
    # bt execution logging
    {
        "comm_type": CommunicationTypes.SUBSCRIPTION,
        "name": "/bt_executor_log",
        "msg_type": ExperimentLogging,
        "callback": "logger_callback",
        "always_on": False,
    },
]
