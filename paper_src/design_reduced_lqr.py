"""Calculate the LQR gain using exactly the same model as MATLAB.

State order:
    z = [theta, theta_dot, r, r_dot]

Control law:
    F = -K @ z
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import solve_continuous_are

from unicycle_full_model import (
    DEFAULT_PARAMETERS,
    UnicycleParameters,
)


# ============================================================
# LQR weights -- identical to MATLAB
# ============================================================

Q = np.diag(
    [
        1000.0,  # theta
        100.0,   # theta_dot
        100.0,   # r
        10.0,    # r_dot
    ]
)

INPUT_WEIGHT = 30.0


def linearize_physical_model(
    params: UnicycleParameters = DEFAULT_PARAMETERS,
) -> tuple[np.ndarray, np.ndarray]:
    """Linearize using the same equations and state order as MATLAB.

    State:
        z = [theta, theta_dot, r, r_dot]

    Input:
        F = lateral force
    """

    g = params.g
    R = params.R

    m_rod = params.mr
    m_B = params.mp
    m_W = params.mw

    I_b = params.JPx
    I_w = params.JW2
    I_rod = params.JR

    h = params.h

    # ========================================================
    # Equilibrium mass matrix
    # ========================================================
    #
    # This is exactly the MATLAB matrix at:
    #
    # theta = theta_dot = r = r_dot = 0

    mass_matrix = np.array(
        [
            [
                m_rod * R**2
                + m_B * (R + h) ** 2
                + m_W * R**2
                + I_b
                + I_w
                + I_rod,
                -m_rod * R,
            ],
            [
                -m_rod * R,
                m_rod,
            ],
        ],
        dtype=float,
    )

    # ========================================================
    # Linearized right-hand-side state coefficients
    # ========================================================
    #
    # State order:
    #
    # z = [theta, theta_dot, r, r_dot]
    #
    # RHS_linear = rhs_state_matrix @ z
    #              + rhs_input_vector * F

    rhs_state_matrix = np.array(
        [
            [
                m_rod * g * R
                + m_B * g * (R + h)
                + m_W * g * R,
                0.0,
                -m_rod * g,
                0.0,
            ],
            [
                -m_rod * g,
                0.0,
                0.0,
                0.0,
            ],
        ],
        dtype=float,
    )

    # In the physical-coordinate MATLAB equations,
    # F appears only in the second equation.

    rhs_input_vector = np.array(
        [
            [0.0],
            [1.0],
        ],
        dtype=float,
    )

    # Solve for:
    #
    # [theta_ddot, r_ddot]^T
    #
    # as functions of z and F.

    acceleration_state_matrix = np.linalg.solve(
        mass_matrix,
        rhs_state_matrix,
    )

    acceleration_input_matrix = np.linalg.solve(
        mass_matrix,
        rhs_input_vector,
    )

    # ========================================================
    # State-space matrices
    # ========================================================

    A = np.zeros(
        (4, 4),
        dtype=float,
    )

    B = np.zeros(
        (4, 1),
        dtype=float,
    )

    # theta_dot
    A[0, 1] = 1.0

    # theta_ddot
    A[1, :] = acceleration_state_matrix[0, :]
    B[1, 0] = acceleration_input_matrix[0, 0]

    # r_dot
    A[2, 3] = 1.0

    # r_ddot
    A[3, :] = acceleration_state_matrix[1, :]
    B[3, 0] = acceleration_input_matrix[1, 0]

    return A, B


def controllability_matrix(
    A: np.ndarray,
    B: np.ndarray,
) -> np.ndarray:
    """Return the continuous-time controllability matrix."""

    return np.hstack(
        [
            B,
            A @ B,
            A @ A @ B,
            A @ A @ A @ B,
        ]
    )


def continuous_lqr_gain(
    A: np.ndarray,
    B: np.ndarray,
    Q: np.ndarray,
    input_weight: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Calculate K for the control law F = -K @ z."""

    R_lqr = np.array(
        [[input_weight]],
        dtype=float,
    )

    P = solve_continuous_are(
        A,
        B,
        Q,
        R_lqr,
    )

    K = np.linalg.solve(
        R_lqr,
        B.T @ P,
    )

    return K, P


def main() -> None:
    params = DEFAULT_PARAMETERS

    A, B = linearize_physical_model(
        params
    )

    controllability = controllability_matrix(
        A,
        B,
    )

    rank = np.linalg.matrix_rank(
        controllability
    )

    if rank < 4:
        raise RuntimeError(
            f"System is not controllable. Rank = {rank}"
        )

    K, P = continuous_lqr_gain(
        A,
        B,
        Q,
        INPUT_WEIGHT,
    )

    closed_loop_poles = np.linalg.eigvals(
        A - B @ K
    )

    # Same gains reordered for:
    #
    # [theta, r, theta_dot, r_dot]

    K_physical_order = K[
        :,
        [0, 2, 1, 3],
    ]

    np.set_printoptions(
        precision=10,
        suppress=True,
    )

    print("========================================")
    print("Full-model parameters")
    print("========================================")
    print(f"g     = {params.g:.10f}")
    print(f"R     = {params.R:.10f}")
    print(f"mrod  = {params.mr:.10f}")
    print(f"m_W   = {params.mw:.10f}")
    print(f"m_B   = {params.mp:.10f}")
    print(f"h     = {params.h:.10f}")
    print(f"I_w   = {params.JW2:.12f}")
    print(f"I_b   = {params.JPx:.12f}")
    print(f"I_rod = {params.JR:.12f}")

    print("\nA matrix:")
    print(A)

    print("\nB matrix:")
    print(B)

    print("\nControllability rank:")
    print(rank)

    print("\n========================================")
    print("MATLAB state order")
    print("z = [theta, theta_dot, r, r_dot]")
    print("F = -K @ z")
    print("========================================")
    print(K)

    print("\nIndividual gains:")
    print(f"K_theta     = {K[0, 0]: .10f}")
    print(f"K_theta_dot = {K[0, 1]: .10f}")
    print(f"K_r         = {K[0, 2]: .10f}")
    print(f"K_r_dot     = {K[0, 3]: .10f}")

    print("\n========================================")
    print("Physical reporting order")
    print("[theta, r, theta_dot, r_dot]")
    print("========================================")
    print(K_physical_order)

    print("\nClosed-loop poles:")
    print(closed_loop_poles)

    print("\nRiccati matrix P:")
    print(P)


if __name__ == "__main__":
    main()