import numpy as np
from Models.Segway_model_nonLinear import Model_nonLinear
from typing import override
from Parameters import params
from Factories import register_model
from Config import MODEL_T


@register_model(MODEL_T.NONLINEAR_DAMP)
class Model_motorDamp(Model_nonLinear):
    def Damping_torque(self, omega):
        # T = B * w + B_0 * sign(w) where w is the angular velocity (of the motor, w = phi'- gamma')
        B = params.B
        B_0 = params.B_0
        return B * omega + B_0 * np.sign(omega)  # Linear + Coulomb damping

    @override
    def state_space(
        self, z, M
    ) -> np.ndarray:  # for non-linear model we have to treat z's elements separately
        _z: np.ndarray = np.asarray(z)
        # first ensure z has four elements
        if _z.shape[0] != 4:
            raise ValueError(
                "State vector z must have four elements: [x, x_dot, gamma, gamma_dot]"
            )

        x, x_dot, gamma, gamma_dot = _z
        m = params.m
        m_w = params.m_w
        h = params.h
        g = params.g
        R = params.R

        M_matrix = np.array(
            [[m + m_w, m * h * np.cos(gamma)], [m * h * np.cos(gamma), m * h**2 ]]
        )

        N = np.linalg.inv(M_matrix)

        # Above are the same as the non-linear model, below we add the motor damping effect

        T = self.Damping_torque(
            x_dot / R - gamma_dot
        )  # omega = phi' - gamma' = x_dot/R - gamma_dot

        x_ddot = N[0, 0] * ((M - T) / R + m * h * gamma_dot**2 * np.sin(gamma)) + N[
            0, 1
        ] * (-(M - T) + m * g * h * np.sin(gamma))

        gamma_ddot = N[1, 0] * (
            (M - T) / R + m * h * gamma_dot**2 * np.sin(gamma)
        ) + N[1, 1] * (-(M - T) + m * g * h * np.sin(gamma))

        return np.array([x_dot, x_ddot, gamma_dot, gamma_ddot])
