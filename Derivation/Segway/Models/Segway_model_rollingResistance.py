import numpy as np
from Models.Segway_model_motorDamp import Model_motorDamp
from typing import override
from Parameters import params
from Factories import register_model
from Config import MODEL_T

@register_model(MODEL_T.ROLLING_RESISTANCE)
class Model_rollingResistance(Model_motorDamp):
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
        m = params.m # neglect inertia of the pendulum
        m_w = params.m_w + params.I / params.R**2  # effective mass of the wheel including its inertia
        h = params.h
        g = params.g
        R = params.R
        mu = params.mu_rolling

        M_matrix = np.array(
            [[m + m_w, m * h * np.cos(gamma)], [m * h * np.cos(gamma), m * h**2 ]]
        )

        N = np.linalg.inv(M_matrix)

        # Above are the same as the non-linear model, below we add the motor damping effect

        T = self.Damping_torque(
            x_dot / R - gamma_dot
        )  # omega = phi' - gamma' = x_dot/R - gamma_dot

        x_ddot = N[0, 0] * ((M - T) / R - mu * (m + m_w) * g * np.sign(x_dot) / R + m * h * gamma_dot**2 * np.sin(gamma)) + N[
            0, 1
        ] * (-(M - T) + m * g * h * np.sin(gamma))

        gamma_ddot = N[1, 0] * (
            (M - T) / R - mu * (m + m_w) * g * np.sign(x_dot) / R + m * h * gamma_dot**2 * np.sin(gamma)
        ) + N[1, 1] * (-(M - T) + m * g * h * np.sin(gamma))

        return np.array([x_dot, x_ddot, gamma_dot, gamma_ddot])