class Parameters:
    def __init__(self):
        # All in SI units
        self.m = 1.0
        self.h = 1.0
        self.R = 0.5
        self.I = self.m * self.h**2
        self.g = 9.81

        self.v_desired = 1.0
        self.gamma_desired = 0.0
        self.s_desired = 0.0

params = Parameters()