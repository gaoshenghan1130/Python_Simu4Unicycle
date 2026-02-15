from Segway_model2 import Model2
import numpy as np

from enum import Enum, auto

class Mode(Enum):
    BALANCE = auto()
    VELOCITY= auto()
    POSITION = auto()

class ModelWithControl(Model2):
    def __init__(self):
        super().__init__()

        # The system model is in the form of z' = A z + B M, where M is the magnitude of the control input (torque applied to the wheel)
        # Here we should define the change of M based on the desired velocity and the current velocity, which is a simple proportional control law. We can later tune the gain to achieve better performance.
        self.control = self.control_law
        self.mode = Mode.BALANCE  # Default to balance mode

    def control_law(self, z, mode):
        # Z is the state vector [x_c, x_c', gamma, gamma']
        z = np.asarray(z)
        x_c = z[0]
        x_c_dot = z[1]
        gamma = z[2]
        gamma_dot = z[3]






        # Simple proportional control law to track the desired velocity

    

    

        


        