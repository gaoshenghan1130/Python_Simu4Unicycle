from Config import CONTROL_T, CONTROL_MODE_T
from Controllers.Register import register_control
from typing import override
from Controllers import Controller


@register_control(CONTROL_T.PD)
class PDController(Controller):
    @override
    def __init__(self, controlMode: CONTROL_MODE_T):
        super().__init__(controlMode)
        self.name = "PD Controller"

    @override
    def compute_control(self, state, reference):
        try:
            q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5 = state
        except ValueError:
            raise ValueError("State must have 10 elements: [q1, q2, q3, q4, q5, dq1, dq2, dq3, dq4, dq5]")

        t_w = 0.0   
        f_l = 0.0
        
        
        
        return {"T_W": t_w, "F_L": f_l}  # Return a dummy control input for now