import numpy as np
from Parameters import params
from Config import CONTROL_MODE, CONTROL_STRATEGY
from Factories import register_controller
from Controllers.Segway_Controller import Controller
from typing import override, Optional


@register_controller(CONTROL_STRATEGY.PID)
class ControllerPID(Controller):
    @override
    def __init__(self, control_mode: CONTROL_MODE):
        super().__init__(control_mode)
        self.vel_error_integral = 0.0
        self.pos_error_integral = 0.0
        self.last_time = None # For dt

    @override 
    def control_law(
        self,
        z,
        t, # for time difference calculation
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
        K_vi = params.K_vi
        K_pi = params.K_pi
        dt : Optional[float] = None

        # Calculate integral error term
        if self.last_time is not None:
            dt = t - self.last_time
            self.vel_error_integral += (x_c_dot - desired_velocity) * dt
            if abs(x_c - desired_position) > params.posDeadZone: # Only integrate position error if it's outside the dead zone
                self.pos_error_integral += (x_c - desired_position) * dt
            self.pos_error_integral = np.clip(self.pos_error_integral, -params.clip_integral, params.clip_integral)
            self.vel_error_integral = np.clip(self.vel_error_integral, -params.clip_integral, params.clip_integral)
        self.last_time = t

        if mode == CONTROL_MODE.BALANCE:
            return +K_gamma * (gamma - desired_gamma) + K_dgamma * gamma_dot
        elif mode == CONTROL_MODE.VELOCITY:
            return (
                +K_gamma * (gamma - desired_gamma)
                + K_dgamma * gamma_dot
                + K_velocity * (x_c_dot - desired_velocity)
                + K_vi * self.vel_error_integral
            )
        elif mode == CONTROL_MODE.POSITION:
            return (
                + K_gamma * (gamma - desired_gamma)
                + K_dgamma * gamma_dot
                + K_velocity * (x_c_dot - desired_velocity)
                + K_position * (x_c - desired_position)
                + K_vi * self.vel_error_integral
                + K_pi * self.pos_error_integral
            )
        else:
            raise ValueError("Invalid control mode")
