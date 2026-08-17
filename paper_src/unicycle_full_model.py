"""Full three-dimensional nonlinear unicycle model."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


STATE_SIZE = 12
INPUT_SIZE = 2

STATE_INDEX = {
    "sigma1": 0,
    "sigma2": 1,
    "sigma3": 2,
    "sigma_r": 3,
    "sigma_g": 4,
    "psi": 5,
    "theta": 6,
    "phi": 7,
    "r": 8,
    "gamma": 9,
    "xG": 10,
    "yG": 11,
}


@dataclass(frozen=True)
class UnicycleParameters:
    """Full-model physical parameters in SI units."""

    g: float = 9.81

    # Wheel
    R: float = 0.253
    mw: float = 2.436
    JW1: float = 0.09099921839
    JW2: float = 0.04591427768

    # Lateral rod / moving mass
    mr: float = 2.3 # After adding the manufactured parts
    JR: float = 0.0517629
    BR: float = 0.0

    # Pendulum / body
    h: float = 0.025
    mp: float = 2.799
    JPx: float = 0.01290418213
    JPy: float = 0.02090219895
    JPz: float = 0.01118711607
    BP: float = 0.0

    def as_array(self) -> NDArray[np.float64]:
        return np.array(
            [
                self.g,
                self.R,
                self.mw,
                self.JW1,
                self.JW2,
                self.mr,
                self.JR,
                self.BR,
                self.h,
                self.mp,
                self.JPx,
                self.JPy,
                self.JPz,
                self.BP,
            ],
            dtype=float,
        )


DEFAULT_PARAMETERS = UnicycleParameters()


def _state_array(state: ArrayLike) -> NDArray[np.float64]:
    state_array = np.asarray(state, dtype=float)

    if state_array.shape != (STATE_SIZE,):
        raise ValueError(
            f"state must have shape ({STATE_SIZE},), "
            f"got {state_array.shape}"
        )

    return state_array


def _input_array(control_input: ArrayLike) -> NDArray[np.float64]:
    input_array = np.asarray(control_input, dtype=float)

    if input_array.shape != (INPUT_SIZE,):
        raise ValueError(
            f"control_input must have shape ({INPUT_SIZE},), "
            f"got {input_array.shape}"
        )

    return input_array


def mass_matrix(
    state: ArrayLike,
    params: UnicycleParameters = DEFAULT_PARAMETERS,
) -> NDArray[np.float64]:
    """Return the 5x5 pseudo-velocity mass matrix."""

    x = _state_array(state)

    (
        sigma1,
        sigma2,
        sigma3,
        sigma_r,
        sigma_g,
        psi,
        theta,
        phi,
        r,
        gamma,
        xG,
        yG,
    ) = x

    del sigma1, sigma2, sigma3
    del sigma_r, sigma_g
    del psi, theta, phi, xG, yG

    (
        g,
        R,
        mw,
        JW1,
        JW2,
        mr,
        JR,
        BR,
        h,
        mp,
        JPx,
        JPy,
        JPz,
        BP,
    ) = params.as_array()

    del g, BR, BP

    sin_gamma = np.sin(gamma)
    cos_gamma = np.cos(gamma)

    inertia_difference = JPx - JPz + h**2 * mp

    matrix = np.zeros((5, 5), dtype=float)

    matrix[0, 0] = (
        JPz
        + JR
        + JW2
        + R**2 * mp
        + R**2 * mw
        + 2.0 * R * h * mp * cos_gamma
        + mr * r**2
        + inertia_difference * cos_gamma**2
    )

    matrix[0, 2] = (
        -R * h * mp * sin_gamma
        - inertia_difference * sin_gamma * cos_gamma
    )
    matrix[2, 0] = matrix[0, 2]

    matrix[1, 1] = JW1 + R**2 * (mp + mr + mw)

    matrix[1, 2] = -R * mr * r
    matrix[2, 1] = matrix[1, 2]

    matrix[1, 4] = R * h * mp * cos_gamma
    matrix[4, 1] = matrix[1, 4]

    matrix[2, 2] = (
        JPz
        + JR
        + JW2
        + mr * r**2
        + inertia_difference * sin_gamma**2
    )

    matrix[3, 3] = mr
    matrix[4, 4] = JPy + h**2 * mp

    return matrix


def residual_vector(
    state: ArrayLike,
    control_input: ArrayLike,
    params: UnicycleParameters = DEFAULT_PARAMETERS,
) -> NDArray[np.float64]:
    """Return C from M @ sigma_dot + C = 0."""

    x = _state_array(state)
    u = _input_array(control_input)

    (
        sigma1,
        sigma2,
        sigma3,
        sigma_r,
        sigma_g,
        psi,
        theta,
        phi,
        r,
        gamma,
        xG,
        yG,
    ) = x

    del phi, xG, yG

    lateral_force, wheel_torque = u

    (
        g,
        R,
        mw,
        JW1,
        JW2,
        mr,
        JR,
        BR,
        h,
        mp,
        JPx,
        JPy,
        JPz,
        BP,
    ) = params.as_array()

    sin = np.sin
    cos = np.cos
    tan = np.tan

    inertia_difference = JPx - JPz + h**2 * mp

    residual = np.empty(STATE_SIZE, dtype=float)

    residual[0] = (
        BR * R**2 * sigma1
        + BR * R * sigma_r
        + lateral_force * R
        - R * g * (mp + mw) * sin(theta)
        + R * mr * r * sigma1**2
        - g * h * mp * sin(theta) * cos(gamma)
        + g * mr * r * cos(theta)
        + 2.0 * mr * r * sigma1 * sigma_r
        + sigma1
        * sigma3
        * (
            R * h * mp * sin(gamma) * tan(theta)
            + inertia_difference
            * sin(gamma)
            * cos(gamma)
            * tan(theta)
        )
        + sigma1
        * sigma_g
        * (
            -2.0 * R * h * mp * sin(gamma)
            - 2.0
            * inertia_difference
            * sin(gamma)
            * cos(gamma)
        )
        + sigma2
        * sigma3
        * (
            -JW1
            - R**2 * mp
            - R**2 * mw
            - R * h * mp * cos(gamma)
            - R * mr * r * tan(theta)
        )
        + sigma3**2
        * (
            R * h * mp * cos(gamma) * tan(theta)
            + mr * r**2 * tan(theta)
            + inertia_difference
            * cos(gamma) ** 2
            * tan(theta)
            + (JPz + JR + JW2) * tan(theta)
        )
        + sigma3
        * sigma_g
        * (
            JPx
            - JPy
            - JPz
            - 2.0 * R * h * mp * cos(gamma)
            - 2.0
            * inertia_difference
            * cos(gamma) ** 2
        )
    )

    residual[1] = (
        BP * sigma2
        - BP * sigma_g
        - wheel_torque
        - R * h * mp * sigma3**2 * sin(gamma)
        - R * h * mp * sigma_g**2 * sin(gamma)
        - 2.0 * R * mr * sigma3 * sigma_r
        + sigma1
        * sigma3
        * (
            R**2 * (mp - mr + mw)
            + R * h * mp * cos(gamma)
            + R * mr * r * tan(theta)
        )
    )

    residual[2] = (
        JW1 * sigma1 * sigma2
        + R * h * mp * sigma2 * sigma3 * sin(gamma)
        + g * h * mp * sin(gamma) * sin(theta)
        + 2.0 * mr * r * sigma3 * sigma_r
        + sigma1
        * sigma3
        * (
            R * mr * r
            - mr * r**2 * tan(theta)
            - inertia_difference
            * sin(gamma) ** 2
            * tan(theta)
            - (JPz + JR + JW2) * tan(theta)
        )
        + sigma1
        * sigma_g
        * (
            -JPx
            + JPy
            + JPz
            + 2.0
            * inertia_difference
            * sin(gamma) ** 2
        )
        - sigma3**2
        * inertia_difference
        * sin(gamma)
        * cos(gamma)
        * tan(theta)
        + 2.0
        * sigma3
        * sigma_g
        * inertia_difference
        * sin(gamma)
        * cos(gamma)
    )

    residual[3] = (
        BR * R * sigma1
        + BR * sigma_r
        + lateral_force
        + R * mr * sigma2 * sigma3
        + g * mr * sin(theta)
        - mr * r * sigma1**2
        - mr * r * sigma3**2
    )

    residual[4] = (
        -BP * sigma2
        + BP * sigma_g
        + wheel_torque
        + R
        * h
        * mp
        * sigma2
        * sigma3
        * sin(gamma)
        * tan(theta)
        - g * h * mp * sin(gamma) * cos(theta)
        + sigma1**2
        * (
            R * h * mp * sin(gamma)
            + inertia_difference
            * sin(gamma)
            * cos(gamma)
        )
        + sigma1
        * sigma3
        * (
            JPx
            - JPz
            + R * h * mp * cos(gamma)
            + h**2 * mp
            - 2.0
            * inertia_difference
            * sin(gamma) ** 2
        )
        - sigma3**2
        * inertia_difference
        * sin(gamma)
        * cos(gamma)
    )

    # Kinematics
    residual[5] = -sigma3 / cos(theta)
    residual[6] = -sigma1
    residual[7] = -sigma2 + sigma3 * tan(theta)
    residual[8] = -R * sigma1 - sigma_r
    residual[9] = sigma3 * tan(theta) - sigma_g

    residual[10] = (
        -R * sigma1 * sin(psi) * cos(theta)
        - R * sigma2 * cos(psi)
    )

    residual[11] = (
        R * sigma1 * cos(psi) * cos(theta)
        - R * sigma2 * sin(psi)
    )

    return residual


def unicycle_full_model(
    t: float,
    state: ArrayLike,
    control_input: ArrayLike,
    params: UnicycleParameters = DEFAULT_PARAMETERS,
) -> NDArray[np.float64]:
    """Evaluate the full-model state derivative."""

    del t

    x = _state_array(state)

    matrix = mass_matrix(x, params)
    derivative = -residual_vector(
        x,
        control_input,
        params,
    )

    derivative[:5] = np.linalg.solve(
        matrix,
        derivative[:5],
    )

    return derivative


def configuration_rates(
    state: ArrayLike,
    params: UnicycleParameters = DEFAULT_PARAMETERS,
) -> NDArray[np.float64]:
    """Return physical generalized-coordinate rates."""

    x = _state_array(state)

    (
        sigma1,
        sigma2,
        sigma3,
        sigma_r,
        sigma_g,
        psi,
        theta,
        phi,
        r,
        gamma,
        xG,
        yG,
    ) = x

    del phi, r, gamma, xG, yG

    R = params.R

    return np.array(
        [
            sigma3 / np.cos(theta),
            sigma1,
            sigma2 - sigma3 * np.tan(theta),
            R * sigma1 + sigma_r,
            -sigma3 * np.tan(theta) + sigma_g,
            R
            * sigma1
            * np.sin(psi)
            * np.cos(theta)
            + R * sigma2 * np.cos(psi),
            -R
            * sigma1
            * np.cos(psi)
            * np.cos(theta)
            + R * sigma2 * np.sin(psi),
        ],
        dtype=float,
    )


def generalized_rates_to_pseudo_velocities(
    configuration_rates_vector: ArrayLike,
    configuration: ArrayLike,
    params: UnicycleParameters = DEFAULT_PARAMETERS,
) -> NDArray[np.float64]:
    """Convert generalized rates to pseudo velocities."""

    q_dot = np.asarray(
        configuration_rates_vector,
        dtype=float,
    )
    q = np.asarray(configuration, dtype=float)

    if q_dot.shape != (7,) or q.shape != (7,):
        raise ValueError(
            "configuration and its rate must both have shape (7,)"
        )

    (
        psi_dot,
        theta_dot,
        phi_dot,
        r_dot,
        gamma_dot,
        xG_dot,
        yG_dot,
    ) = q_dot

    (
        psi,
        theta,
        phi,
        r,
        gamma,
        xG,
        yG,
    ) = q

    del psi, phi, r, gamma, xG, yG
    del xG_dot, yG_dot

    return np.array(
        [
            theta_dot,
            phi_dot + psi_dot * np.sin(theta),
            psi_dot * np.cos(theta),
            -params.R * theta_dot + r_dot,
            gamma_dot + psi_dot * np.sin(theta),
        ],
        dtype=float,
    )


def state_from_configuration(
    configuration: ArrayLike,
    configuration_rates_vector: ArrayLike | None = None,
    params: UnicycleParameters = DEFAULT_PARAMETERS,
) -> NDArray[np.float64]:
    """Build the 12-state vector from physical coordinates."""

    q = np.asarray(configuration, dtype=float)

    if q.shape != (7,):
        raise ValueError(
            "configuration must have shape (7,)"
        )

    if configuration_rates_vector is None:
        q_dot = np.zeros(7, dtype=float)
    else:
        q_dot = np.asarray(
            configuration_rates_vector,
            dtype=float,
        )

    sigmas = generalized_rates_to_pseudo_velocities(
        q_dot,
        q,
        params,
    )

    return np.concatenate((sigmas, q))