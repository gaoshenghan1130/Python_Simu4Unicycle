from Models.Segway_model_motorDamp import Model_motorDamp
from typing import override
from Factories import register_model
from Config import MODEL_T
from Parameters import params
import numpy as np

@register_model(MODEL_T.NONLINEAR_DAMP_SMOOTHER)
class Model_motorDamp_Smoother(Model_motorDamp):
    @override
    def Damping_torque(self, omega):
        B = params.B
        B_0 = params.B_0
        K_tandamp = params.K_tandamp

        return B * omega + B_0 * np.tanh(K_tandamp * omega) # tanh is smoother than sign
