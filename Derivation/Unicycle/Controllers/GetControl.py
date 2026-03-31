from Controller import Controller
from Config import CONTROL_MODE_T, CONTROL_T
from Controllers.Register import control_id_to_class


def get_controller(control_type: CONTROL_T, control_mode: CONTROL_MODE_T) -> Controller:
    if control_type not in control_id_to_class:
        raise ValueError(f"Control type {control_type} is not registered.")
    control_class = control_id_to_class[control_type]
    return control_class(control_mode)

