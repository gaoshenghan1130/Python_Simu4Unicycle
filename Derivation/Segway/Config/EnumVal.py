from enum import Enum, auto


class MODEL_T(int, Enum):
    LINEAR = auto()  # base
    NONLINEAR = auto()
    NONLINEAR_DAMP = auto()
    NONLINEAR_DAMP_SMOOTHER = auto()
    MODEL1 = auto()
    MODEL2 = auto()


class CONTROL_MODE(int, Enum):
    BALANCE = auto()  # base
    VELOCITY = auto()
    POSITION = auto()


class CONTROL_STRATEGY(int, Enum):
    PD = auto()  # base
    PID = auto()
    LQR = auto()
