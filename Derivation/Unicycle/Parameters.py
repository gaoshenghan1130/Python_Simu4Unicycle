class Parameters:
    def __init__(self):
        # All in SI units
        self.m = 1  # pendulum mass
        self.m_w = 1.5  # wheel mass
        self.m_l = 0.5  # linear motor mass
        self.h = 0.1
        self.R = 0.25
        self.I = 0.0
        self.g = 9.81


params = Parameters()  # Light singleton pattern

__all__ = ["params"]
