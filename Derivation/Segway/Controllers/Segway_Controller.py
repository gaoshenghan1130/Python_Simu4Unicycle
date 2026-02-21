import numpy as np
from Parameters import params
from Config import CONTROL_MODE, CONTROL_STRATEGY
from Factories import register_controller


@register_controller(CONTROL_STRATEGY.PD)
class Controller:
    def __init__(self, control_mode: CONTROL_MODE):
        self.control_mode = control_mode

    def control_law(
        self,
        z,
        desired_gamma=0.0,
        desired_velocity=0.0,
        desired_position=0.0,
    ):
        # Z is the state vector [x_c, x_c', gamma, gamma']
        _z = np.asarray(z)
        if _z.shape[0] != 4:
            raise ValueError(
                "State vector z must have four elements: [x_c, x_c_dot, gamma, gamma_dot]"
            )
        mode = self.control_mode
        x_c = _z[0]
        x_c_dot = _z[1]
        gamma = _z[2]
        gamma_dot = _z[3]
        K_gamma = params.K_gamma
        K_dgamma = params.K_dgamma
        K_velocity = params.K_velocity
        K_position = params.K_position

        if mode == CONTROL_MODE.BALANCE:
            return +K_gamma * (gamma - desired_gamma) + K_dgamma * gamma_dot
        elif mode == CONTROL_MODE.VELOCITY:
            return (
                +K_gamma * (gamma - desired_gamma)
                + K_dgamma * gamma_dot
                + K_velocity * (x_c_dot - desired_velocity)
            )
        elif mode == CONTROL_MODE.POSITION:
            return (
                +K_gamma * (gamma - desired_gamma)
                + K_dgamma * gamma_dot
                + K_velocity * (x_c_dot - desired_velocity)
                + K_position * (x_c - desired_position)
            )
        else:
            raise ValueError("Invalid control mode")
