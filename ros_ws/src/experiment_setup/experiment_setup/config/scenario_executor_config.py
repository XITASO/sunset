from python_base_class.node_config import CommunicationTypes
from system_interfaces.msg import ExperimentLogging
from system_interfaces.srv import SetHeartbeatStatus, SetFlightPhase

comm_types = [
    {
        "comm_type": CommunicationTypes.PUBLISHER,
        "name": "/scenario_executor_log",
        "msg_type": ExperimentLogging,
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.PUBLISHER,
        "name": "/bt_executor_log",
        "msg_type": ExperimentLogging,
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.SERVICE_CLIENT,
        "name": "/degrade_camera_heartbeat",
        "srv_type": SetHeartbeatStatus,
        "always_on": False,
    },
    {
        "comm_type": CommunicationTypes.SERVICE_CLIENT,
        "name": "/set_flight_phase",
        "srv_type": SetFlightPhase,
        "always_on": False,
    },
]
