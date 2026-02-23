from Parameters import params
import numpy as np
from typing import override
from Models.Segway_model import Model
from Factories import register_model
from Config import MODEL_T

@register_model(MODEL_T.NONLINEAR)
class Model_nonLinear(Model):
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
        I = params.I
        g = params.g
        R = params.R

        # Mass matrix on the left-hand side
        M_matrix = np.array(
            [[m + m_w, m * h * np.cos(gamma)], [m * h * np.cos(gamma), m * h**2 ]]
        )

        N = np.linalg.inv(M_matrix)  # to move it to the right-hand side

        x_ddot = N[0, 0] * (M / R + m * h * gamma_dot**2 * np.sin(gamma)) + N[
            0, 1
        ] * (-M + m * g * h * np.sin(gamma))

        gamma_ddot = N[1, 0] * (M / R + m * h * gamma_dot**2 * np.sin(gamma)) + N[
            1, 1
        ] * (-M + m * g * h * np.sin(gamma))

        return np.array([x_dot, x_ddot, gamma_dot, gamma_ddot])

