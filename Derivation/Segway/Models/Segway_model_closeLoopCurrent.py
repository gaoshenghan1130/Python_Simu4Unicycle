import numpy as np
from Models.Segway_model_motorDamp import Model_motorDamp
from typing_extensions import override
from Parameters import params
from Factories import register_model
from Config import MODEL_T

@register_model(MODEL_T.ROLLING_RESISTANCE_LATENCY) # Suggest adding a new config type
class Model_rollingResistanceLatency(Model_motorDamp):
    @override
    def state_space(
        self, z, M
    ) -> np.ndarray:
        """
        State-space model considering Rolling Resistance and Motor Latency.
        z = [x, x_dot, gamma, gamma_dot, M_actual]
        """
        _z: np.ndarray = np.asarray(z)
        
        # Check for 5 states now (added M_actual)
        if _z.shape[0] != 5:
            raise ValueError(
                "State vector z must have five elements: [x, x_dot, gamma, gamma_dot, M_actual]"
            )

        x, x_dot, gamma, gamma_dot, M_actual = _z
        
        # Physical Parameters
        m = params.m 
        m_w = params.m_w + params.I / params.R**2
        h = params.h
        g = params.g
        R = params.R
        mu = params.mu_rolling
        smooth_factor = params.smooth_factor
        tau = params.tau_motor  # Motor time constant (L/R)

        # Mass Matrix
        M_matrix = np.array(
            [[m + m_w, m * h * np.cos(gamma)], 
             [m * h * np.cos(gamma), m * h**2]]
        )
        N = np.linalg.inv(M_matrix)

        # Damping torque (internal motor friction/back-EMF effects)
        T_damp = self.Damping_torque(x_dot / R - gamma_dot)

        # Rolling resistance force
        rollingResistance = mu * (m + m_w) * g * np.tanh(smooth_factor * x_dot) / R

        # Equations of Motion using M_actual (the delayed torque)
        # Replacing 'M' from your original code with 'M_actual'
        x_ddot = N[0, 0] * ((M_actual - T_damp) / R - rollingResistance + m * h * gamma_dot**2 * np.sin(gamma)) + \
                 N[0, 1] * (-(M_actual - T_damp) + m * g * h * np.sin(gamma))

        gamma_ddot = N[1, 0] * ((M_actual - T_damp) / R - rollingResistance + m * h * gamma_dot**2 * np.sin(gamma)) + \
                     N[1, 1] * (-(M_actual - T_damp) + m * g * h * np.sin(gamma))

        # Torque Derivative (First order lag)
        M_dot = (1.0 / tau) * (M - M_actual)

        return np.array([x_dot, x_ddot, gamma_dot, gamma_ddot, M_dot])