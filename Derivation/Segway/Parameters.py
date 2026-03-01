class Parameters:
    def __init__(self):
        # All in SI units
        self.m = 1
        self.m_w = 1.5
        self.h = 0.1
        self.R = 0.25
        self.I = 0.0
        self.g = 9.81

        self.v_desired = 1.0
        self.gamma_desired = 0.0
        self.s_desired = 0.0

        # for motor damping
        self.B = 0
        self.B_0 = 0
        self.K_tandamp = 10.0  # for smoothing the damping torque

        # for PD controller
        self.K_gamma = 3.0
        self.K_dgamma = 0.8
        self.K_velocity = 4.0
        self.K_position = 2

        # for PID controller
        self.K_vi = 0.1
        self.K_pi = 0.05
        self.posDeadZone = 0.005
        self.clip_integral = 1.0


params = Parameters()  # Light singleton pattern

__all__ = ["params"]
