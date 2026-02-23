from Parameters import params
import numpy as np
from typing_extensions import override
from Factories import register_model
from Config import MODEL_T

@register_model(MODEL_T.LINEAR)
class Model:
    def __init__(self):
        # Main equation: Z' = A Z + B M, where Z = [x, x', gamma, gamma']^T
        m = params.m
        m_w = params.m_w
        h = params.h
        I = params.I
        g = params.g
        R = params.R

        # Linearized mass matrix at equilibrium on the left-hand side
        M0 = np.array([[m + m_w / R**2, m * h], [m * h, m * h**2 ]])

        N = np.linalg.inv(M0)  # to move it to the right-hand side

        # ---- A matrix ----
        A = np.zeros((4, 4))

        A[0, 1] = 1
        A[2, 3] = 1

        A[1, 2] = N[0, 1] * m * g * h
        A[3, 2] = N[1, 1] * m * g * h

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

        # Stability check, reverted pendulum should be unstable, so we expect at least one eigenvalue with positive real part
        if np.any(np.real(eigvals) > 0):
            print("[Unstable]: PASSED!")
        else:
            raise ValueError("The system is stable, but it should be unstable. Please check the model equations and parameters.")
        
        # Controllability check, we expect the system to be controllable, so the controllability matrix should have full rank - Suggested by Yang
        controllability_matrix = np.hstack([self.B, self.A @ self.B, self.A @ self.A @ self.B, self.A @ self.A @ self.A @ self.B])
        rank = np.linalg.matrix_rank(controllability_matrix)
        if rank == 4:
            print("[Controllability]: PASSED!")
        else:
            print("Controllability rank is:", rank)
            raise ValueError("The system is not controllable, the rank of the controllability matrix is less than 4.")

    def state_space(self, z, M) -> np.ndarray:
        z = np.asarray(z)
        return self.A @ z + self.B[:, 0] * M

