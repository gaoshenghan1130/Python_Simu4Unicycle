from enum import Enum


class MODEL_T(int, Enum):
    LINEAR = 1 # base
    NONLINEAR = 2
    MODEL1 = 3
    MODEL2 = 4


class CONTROL_MODE(int, Enum):
    BALANCE = 1
    VELOCITY = 2
    POSITION = 3


class CONTROL_STRATEGY(int, Enum):
    PD = 1 # base
    LQR = 2
