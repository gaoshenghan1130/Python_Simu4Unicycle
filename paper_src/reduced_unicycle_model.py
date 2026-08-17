"""Reduced four-state lateral unicycle model."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from unicycle_full_model import (
    DEFAULT_PARAMETERS,
    UnicycleParameters,
)


REDUCED_STATE_SIZE = 4

REDUCED_STATE_INDEX = {
    "theta": 0,
    "q2": 1,
    "theta_dot": 2,
    "u2": 3,
}


@dataclass(frozen=True)
class ReducedUnicycleParameters:
    """Reduced-model parameters."""

    g: float
    R: float
    mrod: float
    J: float
    G: float

    @classmethod
    def from_full_model(
        cls,
        full: UnicycleParameters = DEFAULT_PARAMETERS,
    ) -> "ReducedUnicycleParameters":
        """Generate consistent parameters from the full model."""

        constant_inertia = (
            full.mw * full.R**2
            + full.mp * (full.R + full.h) ** 2
            + full.JW2
            + full.JPx
            + full.JR
        )

        gravity_coefficient = (
            full.mw * full.g * full.R
            + full.mp
            * full.g
            * (full.R + full.h)
        )

        return cls(
            g=full.g,
            R=full.R,
            mrod=full.mr,
            J=constant_inertia,
            G=gravity_coefficient,
        )


# Used for a fair comparison with the full model.
DEFAULT_REDUCED_PARAMETERS = (
    ReducedUnicycleParameters.from_full_model()
)


# Parameters printed in the old report.
REPORT_REDUCED_PARAMETERS = ReducedUnicycleParameters(
    g=9.81,
    R=0.2527,
    mrod=0.78,
    J=0.850948883808,
    G=23.4657162,
)


def _state_array(
    state: ArrayLike,
) -> NDArray[np.float64]:
    state_array = np.asarray(state, dtype=float)

    if state_array.shape != (REDUCED_STATE_SIZE,):
        raise ValueError(
            f"state must have shape "
            f"({REDUCED_STATE_SIZE},), "
            f"got {state_array.shape}"
        )

    return state_array


def reduced_unicycle_model(
    t: float,
    state: ArrayLike,
    lateral_force: float,
    params: ReducedUnicycleParameters
    = DEFAULT_REDUCED_PARAMETERS,
) -> NDArray[np.float64]:
    """Evaluate the reduced nonlinear state derivative.

    State:
        x = [theta, q2, theta_dot, u2]

    Coordinates:
        q2 = r - R * theta
        u2 = r_dot - R * theta_dot
    """

    del t

    theta, q2, theta_dot, u2 = _state_array(
        state
    )

    force = float(lateral_force)

    r = q2 + params.R * theta

    angular_inertia = (
        params.J
        + params.mrod * r**2
    )

    theta_acceleration = (
        force * params.R
        - params.mrod
        * params.g
        * r
        * np.cos(theta)
        + params.G * np.sin(theta)
        - params.mrod
        * r
        * (
            2.0 * theta_dot * u2
            + params.R * theta_dot**2
        )
    ) / angular_inertia

    u2_acceleration = (
        force
        - params.mrod
        * params.g
        * np.sin(theta)
        + params.mrod
        * r
        * theta_dot**2
    ) / params.mrod

    return np.array(
        [
            theta_dot,
            u2,
            theta_acceleration,
            u2_acceleration,
        ],
        dtype=float,
    )


def reduced_state_from_physical(
    theta: float,
    r: float,
    theta_dot: float = 0.0,
    r_dot: float = 0.0,
    params: ReducedUnicycleParameters
    = DEFAULT_REDUCED_PARAMETERS,
) -> NDArray[np.float64]:
    """Convert physical states to reduced coordinates."""

    q2 = r - params.R * theta
    u2 = r_dot - params.R * theta_dot

    return np.array(
        [
            theta,
            q2,
            theta_dot,
            u2,
        ],
        dtype=float,
    )


def physical_lateral_state(
    state: ArrayLike,
    params: ReducedUnicycleParameters
    = DEFAULT_REDUCED_PARAMETERS,
) -> NDArray[np.float64]:
    """Return [theta, r, theta_dot, r_dot]."""

    theta, q2, theta_dot, u2 = _state_array(
        state
    )

    r = q2 + params.R * theta
    r_dot = u2 + params.R * theta_dot

    return np.array(
        [
            theta,
            r,
            theta_dot,
            r_dot,
        ],
        dtype=float,
    )