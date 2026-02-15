from Parameters import params
import numpy as np


class Model1:
    def __init__(self):
        # Main equation: Z' = A Z + B M, where Z = [x, x', gamma, gamma']^T
        m = params.m
        m_w = params.m_w
        h = params.h
        I = params.I
        I_w = params.I_w
        g = params.g
        R = params.R

        # Linearized mass matrix at equilibrium on the left-hand side
        M0 = np.array([[m + m_w + I_w / R**2, m * h], [m * h, m * h**2 + I ]])

        N = np.linalg.inv(M0)  # to move it to the right-hand side

        # ---- A matrix ----
        A = np.zeros((4, 4))

        A[0, 1] = 1
        A[2, 3] = 1

        A[1, 2] = -N[0, 1] * m * g * h
        A[3, 2] = -N[1, 1] * m * g * h

        # ---- B matrix ----
        B = np.zeros((4, 1))

        B[1, 0] = N[0, 0] / R - N[0, 1]
        B[3, 0] = N[1, 0] / R - N[1, 1]

        self.A = A
        self.B = B

    def Check(self) -> None:
        # check eigenvalues of A for stability
        eigvals = np.linalg.eigvals(self.A)
        print("Eigenvalues for A:", eigvals)

    def state_space(self, z, M) -> np.ndarray:
        z = np.asarray(z)
        return self.A @ z + self.B[:, 0] * M


Segway_model1 = Model1()  # Light singleton pattern
