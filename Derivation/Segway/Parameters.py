class Parameters:
    def __init__(self):
        # All in SI units
        self.m = 3
        self.m_w = 4
        self.h = 0.2
        self.R = 0.5
        self.I = self.m * self.h * self.h # Assuming the body is a rod pivoting at one end
        self.I_w = 0.5 * self.m_w * self.R * self.R # Assuming the wheel is a solid disk
        self.g = 9.81

        self.v_desired = 1.0
        self.gamma_desired = 0.0
        self.s_desired = 0.0

        self.K_gamma = 100.0
        self.K_dgamma = 0.8
        self.K_velocity = 0.4
        self.K_position = 2.0

params = Parameters()