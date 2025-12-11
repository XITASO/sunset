from enum import Enum

class CommunicationTypes(Enum):
    PUBLISHER = 0
    SUBSCRIPTION = 1
    SERVICE = 2
    SERVICE_CLIENT = 3
    ACTION_SERVER = 4
    ACTION_CLIENT = 5
