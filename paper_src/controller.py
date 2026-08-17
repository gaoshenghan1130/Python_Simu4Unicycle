"""Composable controllers for the full and reduced unicycle models."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from reduced_unicycle_model import (
    DEFAULT_REDUCED_PARAMETERS,
    ReducedUnicycleParameters,
    physical_lateral_state,
)
from unicycle_full_model import (
    DEFAULT_PARAMETERS,
    STATE_INDEX,
    UnicycleParameters,
)


# ============================================================
# Common controller states
# ============================================================


@dataclass(frozen=True)
class LateralState:
    """Physical lateral state supplied to every lateral controller.

    State order:
        [theta, r, theta_dot, r_dot]
    """

    theta: float
    r: float
    theta_dot: float
    r_dot: float


@dataclass(frozen=True)
class GammaState:
    """Physical longitudinal body-angle state."""

    gamma: float
    gamma_dot: float


# ============================================================
# Controller interfaces
# ============================================================


LateralController = Callable[
    [
        float,
        LateralState,
    ],
    float,
]

GammaController = Callable[
    [
        float,
        GammaState,
    ],
    float,
]

FullModelController = Callable[
    [
        float,
        ArrayLike,
        UnicycleParameters,
    ],
    NDArray[np.float64],
]

ReducedModelController = Callable[
    [
        float,
        ArrayLike,
        ReducedUnicycleParameters,
    ],
    float,
]


# ============================================================
# Lateral PD/LQR controller
# ============================================================


@dataclass(frozen=True)
class PDGains:
    """Lateral PD/LQR gains and references.

    Physical state order:
        [theta, r, theta_dot, r_dot]

    Equivalent MATLAB state order:
        [theta, theta_dot, r, r_dot]
    """

    kp_theta: float = -806.66155536
    kd_theta: float = -144.58221828

    kp_r: float = 885.34727354
    kd_r: float = 155.84633817

    theta_reference: float = 0.0
    r_reference: float = 0.0

    force_limit: float = np.inf


DEFAULT_PD_GAINS = PDGains()


def pd_lateral_force(
    lateral_state: LateralState,
    gains: PDGains = DEFAULT_PD_GAINS,
) -> float:
    """Evaluate the lateral PD/LQR law.

    The returned force uses the reduced/Appell force direction F.

    Control law:
        F = kp_theta*(theta_ref - theta)
            - kd_theta*theta_dot
            + kp_r*(r_ref - r)
            - kd_r*r_dot
    """

    force = (
        gains.kp_theta
        * (
            gains.theta_reference
            - lateral_state.theta
        )
        - gains.kd_theta
        * lateral_state.theta_dot
        + gains.kp_r
        * (
            gains.r_reference
            - lateral_state.r
        )
        - gains.kd_r
        * lateral_state.r_dot
    )

    m_rod = DEFAULT_PARAMETERS.mr
    gravity = DEFAULT_PARAMETERS.g

    #force = m_rod * gravity * np.sin(lateral_state.theta) - 1000 * lateral_state.r - 0 * lateral_state.theta_dot - 155 * lateral_state.r_dot


    force = np.clip(
        force,
        -gains.force_limit,
        gains.force_limit,
    )

    return float(force)


def make_pd_lateral_controller(
    gains: PDGains = DEFAULT_PD_GAINS,
) -> LateralController:
    """Create a lateral PD/LQR controller callable."""

    def controller(
        t: float,
        state: LateralState,
    ) -> float:

        del t

        return pd_lateral_force(
            state,
            gains,
        )

    return controller


def zero_lateral_controller(
    t: float,
    state: LateralState,
) -> float:
    """Return zero lateral force for open-loop testing."""

    del t, state

    return 0.0


# ============================================================
# Longitudinal gamma PD controller
# ============================================================


@dataclass(frozen=True)
class GammaPDGains:
    """Longitudinal gamma PD gains and references."""

    kp_gamma: float = 3.0
    kd_gamma: float = 0.8

    gamma_reference: float = 0.0
    gamma_dot_reference: float = 0.0

    torque_limit: float = np.inf


DEFAULT_GAMMA_PD_GAINS = GammaPDGains()


def gamma_pd_torque(
    gamma_state: GammaState,
    gains: GammaPDGains
    = DEFAULT_GAMMA_PD_GAINS,
) -> float:
    """Evaluate the gamma PD wheel-torque controller.

    With the full-model sign convention, positive M2 produces
    negative gamma acceleration. Therefore positive gains multiply
    gamma error and gamma-rate error directly.

    Control law:
        M2 = kp_gamma*(gamma - gamma_ref)
             + kd_gamma*(gamma_dot - gamma_dot_ref)
    """

    torque = (
        gains.kp_gamma
        * (
            gamma_state.gamma
            - gains.gamma_reference
        )
        + gains.kd_gamma
        * (
            gamma_state.gamma_dot
            - gains.gamma_dot_reference
        )
    )

    torque = np.clip(
        torque,
        -gains.torque_limit,
        gains.torque_limit,
    )

    return float(torque)


def make_gamma_pd_controller(
    gains: GammaPDGains
    = DEFAULT_GAMMA_PD_GAINS,
) -> GammaController:
    """Create a gamma PD controller callable."""

    def controller(
        t: float,
        state: GammaState,
    ) -> float:

        del t

        return gamma_pd_torque(
            state,
            gains,
        )

    return controller


def zero_gamma_controller(
    t: float,
    state: GammaState,
) -> float:
    """Return zero longitudinal wheel torque."""

    del t, state

    return 0.0


# ============================================================
# Full-model state extraction
# ============================================================


def full_physical_lateral_state(
    state: ArrayLike,
    params: UnicycleParameters
    = DEFAULT_PARAMETERS,
) -> LateralState:
    """Extract physical lateral states from the full model."""

    x = np.asarray(
        state,
        dtype=float,
    )

    theta = float(
        x[STATE_INDEX["theta"]]
    )

    r = float(
        x[STATE_INDEX["r"]]
    )

    theta_dot = float(
        x[STATE_INDEX["sigma1"]]
    )

    r_dot = float(
        params.R * theta_dot
        + x[STATE_INDEX["sigma_r"]]
    )

    return LateralState(
        theta=theta,
        r=r,
        theta_dot=theta_dot,
        r_dot=r_dot,
    )


def full_physical_gamma_state(
    state: ArrayLike,
) -> GammaState:
    """Extract gamma and gamma_dot from the full model."""

    x = np.asarray(
        state,
        dtype=float,
    )

    theta = x[
        STATE_INDEX["theta"]
    ]

    gamma = float(
        x[STATE_INDEX["gamma"]]
    )

    gamma_dot = float(
        -x[STATE_INDEX["sigma3"]]
        * np.tan(theta)
        + x[STATE_INDEX["sigma_g"]]
    )

    return GammaState(
        gamma=gamma,
        gamma_dot=gamma_dot,
    )


# ============================================================
# Reduced-model state extraction
# ============================================================


def reduced_physical_lateral_state(
    state: ArrayLike,
    params: ReducedUnicycleParameters
    = DEFAULT_REDUCED_PARAMETERS,
) -> LateralState:
    """Extract physical lateral states from the reduced model."""

    (
        theta,
        r,
        theta_dot,
        r_dot,
    ) = physical_lateral_state(
        state,
        params,
    )

    return LateralState(
        theta=float(theta),
        r=float(r),
        theta_dot=float(theta_dot),
        r_dot=float(r_dot),
    )


# ============================================================
# Model-interface adapters
# ============================================================


def make_full_model_controller(
    lateral_controller: LateralController,
    gamma_controller: GammaController,
) -> FullModelController:
    """Adapt independent controllers to full input [F1, M2].

    The supplied lateral controller always returns force F in
    the reduced/Appell direction.

    The full symbolic model uses:
        F1 = -F

    The gamma controller returns full-model wheel torque M2.
    """

    def controller(
        t: float,
        state: ArrayLike,
        params: UnicycleParameters
        = DEFAULT_PARAMETERS,
    ) -> NDArray[np.float64]:

        lateral_state = (
            full_physical_lateral_state(
                state,
                params,
            )
        )

        gamma_state = (
            full_physical_gamma_state(
                state,
            )
        )

        lateral_force = float(
            lateral_controller(
                t,
                lateral_state,
            )
        )

        wheel_torque = float(
            gamma_controller(
                t,
                gamma_state,
            )
        )

        if not np.isfinite(
            lateral_force
        ):
            raise ValueError(
                "Lateral controller returned NaN or Inf."
            )

        if not np.isfinite(
            wheel_torque
        ):
            raise ValueError(
                "Gamma controller returned NaN or Inf."
            )

        # Convert the common lateral force F to
        # the full-model force direction F1.

        F1 = -lateral_force
        M2 = wheel_torque

        return np.array(
            [
                F1,
                M2,
            ],
            dtype=float,
        )

    return controller


def make_reduced_model_controller(
    lateral_controller: LateralController,
) -> ReducedModelController:
    """Adapt a common lateral law to the reduced-model force F."""

    def controller(
        t: float,
        state: ArrayLike,
        params: ReducedUnicycleParameters
        = DEFAULT_REDUCED_PARAMETERS,
    ) -> float:

        lateral_state = (
            reduced_physical_lateral_state(
                state,
                params,
            )
        )

        force = float(
            lateral_controller(
                t,
                lateral_state,
            )
        )

        if not np.isfinite(force):
            raise ValueError(
                "Lateral controller returned NaN or Inf."
            )

        return force

    return controller


# ============================================================
# Default controller selection
# ============================================================


DEFAULT_LATERAL_CONTROLLER = (
    make_pd_lateral_controller(
        DEFAULT_PD_GAINS
    )
)

DEFAULT_GAMMA_CONTROLLER = (
    make_gamma_pd_controller(
        DEFAULT_GAMMA_PD_GAINS
    )
)


# Default full-model controller:
#
# input  = 12-state full-model state
# output = [F1, M2]

full_model_controller = (
    make_full_model_controller(
        DEFAULT_LATERAL_CONTROLLER,
        DEFAULT_GAMMA_CONTROLLER,
    )
)


# Default reduced-model controller:
#
# input  = four-state reduced-model state
# output = F

reduced_model_controller = (
    make_reduced_model_controller(
        DEFAULT_LATERAL_CONTROLLER
    )
)


# Backward-compatible name
controller = full_model_controller


__all__ = [
    "DEFAULT_GAMMA_CONTROLLER",
    "DEFAULT_GAMMA_PD_GAINS",
    "DEFAULT_LATERAL_CONTROLLER",
    "DEFAULT_PD_GAINS",
    "FullModelController",
    "GammaController",
    "GammaPDGains",
    "GammaState",
    "LateralController",
    "LateralState",
    "PDGains",
    "ReducedModelController",
    "controller",
    "full_model_controller",
    "full_physical_gamma_state",
    "full_physical_lateral_state",
    "gamma_pd_torque",
    "make_full_model_controller",
    "make_gamma_pd_controller",
    "make_pd_lateral_controller",
    "make_reduced_model_controller",
    "pd_lateral_force",
    "reduced_model_controller",
    "reduced_physical_lateral_state",
    "zero_gamma_controller",
    "zero_lateral_controller",
]