from enum import Enum, auto


class MODEL_T(int, Enum):
    EMPTY = auto()
    KANE = auto()
    APPEl = auto()
    
class CONTROL_T(int, Enum):
    EMPTY = auto()
    PD = auto()

class CONTROL_MODE_T(int, Enum):
    BALANCE = auto()
    VELOCITY = auto()
    POSITION = auto()

