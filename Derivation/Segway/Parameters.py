class Parameters:
    def __init__(self):
        # All in SI units
        self.m = 3
        self.m_w = 4
        self.h = 0.2
        self.R = 0.5
        self.I = self.m * self.h * self.h # Assuming the body is a rod pivoting at one end 
        self.g = 9.81

        self.v_desired = 1.0
        self.gamma_desired = 0.0
        self.s_desired = 0.0

        # for motor damping
        self.B = 0.1
        self.B_0 = 0.2
        self.K_tandamp = 10.0 # for smoothing the damping torque

        # for PD controller
        self.K_gamma = 100.0
        self.K_dgamma = 0.8
        self.K_velocity = 0.4
        self.K_position = 2.0

        # for PID controller
        self.K_vi = 0.1
        self.K_pi = 0.05
        self.posDeadZone = 0.005
        self.clip_integral = 1.0

params = Parameters() # Light singleton pattern

__all__ = ["params"]