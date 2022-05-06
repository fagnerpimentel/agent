from enum import Enum

class LogLevel(Enum):
    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3
    FATAL = 4

class Side(Enum):
    LEFT = -1
    RIGHT = 1

class DeviceType(Enum):
    SENSOR = 0
    ACTUATOR = 1

class Status(Enum):
    NOT_FOUND = 0
    FOUND = 1
    OK = 2

class ROSComunicationType(Enum):
    TOPIC = 0
    SERVICE = 1
    ACTION = 2

class ActionlibState(Enum):
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    LOST = 6

class LogColor(Enum):
    DEFAULT = ['white','black']
    RED = ['red','red']
    GREEN = ['green','OliveGreen']
    BLUE = ['blue','blue']
    YELLOW = ['yellow','YellowOrange']
    MAGENTA = ['magenta','magenta']
    CYAN = ['cyan','Cyan']
