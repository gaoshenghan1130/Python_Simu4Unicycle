from typing import Dict
from Controllers.Register import register_control
from Config import CONTROL_T, CONTROL_MODE_T


@register_control(CONTROL_T.EMPTY)
class Controller:
    def __init__(self, controlMode: CONTROL_MODE_T):
        self.controlMode = controlMode
        self.name = "Empty Controller"
        
    def compute_control(self, state, reference) -> Dict[str, float]:
        return {"T_W": 0.0, "F_L": 0.0}  # non force input
