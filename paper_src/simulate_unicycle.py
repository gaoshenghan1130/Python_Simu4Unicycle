"""Compare the full and reduced nonlinear unicycle models."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.integrate import solve_ivp

from controller import (
    full_model_controller,
    reduced_model_controller,
)
from reduced_unicycle_model import (
    DEFAULT_REDUCED_PARAMETERS,
    ReducedUnicycleParameters,
    physical_lateral_state,
    reduced_state_from_physical,
    reduced_unicycle_model,
)
from unicycle_full_model import (
    DEFAULT_PARAMETERS,
    STATE_INDEX,
    UnicycleParameters,
    state_from_configuration,
    unicycle_full_model,
)


FullController = Callable[
    [
        float,
        NDArray[np.float64],
        UnicycleParameters,
    ],
    ArrayLike,
]

ReducedController = Callable[
    [
        float,
        NDArray[np.float64],
        ReducedUnicycleParameters,
    ],
    float,
]


@dataclass(frozen=True)
class InitialCondition:
    """Physical initial conditions."""

    # Lateral states
    theta: float = np.deg2rad(1)
    r: float = 0.0
    theta_dot: float = 0.0
    r_dot: float = 0.0

    # Longitudinal states for the full model
    longitudinal_velocity: float = 0.2
    gamma: float = 0.1
    gamma_dot: float = 0.0


DEFAULT_INITIAL_CONDITION = InitialCondition()


def full_initial_state(
    initial: InitialCondition
    = DEFAULT_INITIAL_CONDITION,
    params: UnicycleParameters
    = DEFAULT_PARAMETERS,
) -> NDArray[np.float64]:
    """Build the full-model initial state."""

    # Full-model configuration:
    #
    # q = [
    #     psi,
    #     theta,
    #     phi,
    #     r,
    #     gamma,
    #     xG,
    #     yG,
    # ]

    configuration = np.array(
        [
            0.0,             # psi
            initial.theta,   # theta
            0.0,             # phi
            initial.r,       # r
            initial.gamma,   # gamma
            0.0,             # xG
            0.0,             # yG
        ],
        dtype=float,
    )

    # At zero yaw rate:
    #
    # longitudinal_velocity = R * phi_dot

    phi_dot = (
        initial.longitudinal_velocity
        / params.R
    )

    configuration_rates = np.array(
        [
            0.0,                            # psi_dot
            initial.theta_dot,              # theta_dot
            phi_dot,                        # phi_dot
            initial.r_dot,                  # r_dot
            initial.gamma_dot,              # gamma_dot
            initial.longitudinal_velocity,  # xG_dot
            0.0,                            # yG_dot
        ],
        dtype=float,
    )

    return state_from_configuration(
        configuration,
        configuration_rates,
        params,
    )


def sample_times(
    t_span: tuple[float, float],
    output_step: float,
) -> NDArray[np.float64]:
    """Generate common output times."""

    if output_step <= 0.0:
        raise ValueError(
            "output_step must be positive."
        )

    duration = (
        t_span[1]
        - t_span[0]
    )

    count = max(
        2,
        int(
            np.ceil(
                duration / output_step
            )
        ) + 1,
    )

    return np.linspace(
        t_span[0],
        t_span[1],
        count,
    )


def simulate_full_model(
    initial: InitialCondition
    = DEFAULT_INITIAL_CONDITION,
    t_span: tuple[float, float]
    = (0.0, 15.0),
    controller_function: FullController
    = full_model_controller,
    params: UnicycleParameters
    = DEFAULT_PARAMETERS,
    max_step: float = 0.005,
    output_step: float = 0.005,
) -> Any:
    """Simulate the full 12-state model."""

    initial_state = full_initial_state(
        initial,
        params,
    )

    def closed_loop_rhs(
        t: float,
        state: NDArray[np.float64],
    ) -> NDArray[np.float64]:

        control_input = controller_function(
            t,
            state,
            params,
        )

        return unicycle_full_model(
            t,
            state,
            control_input,
            params,
        )

    def theta_limit_event(
        t: float,
        state: NDArray[np.float64],
    ) -> float:

        del t

        theta = state[
            STATE_INDEX["theta"]
        ]

        return (
            np.deg2rad(75.0)
            - abs(theta)
        )

    theta_limit_event.terminal = True
    theta_limit_event.direction = -1

    return solve_ivp(
        closed_loop_rhs,
        t_span,
        initial_state,
        method="RK45",
        t_eval=sample_times(
            t_span,
            output_step,
        ),
        rtol=1.0e-8,
        atol=1.0e-10,
        max_step=max_step,
        events=theta_limit_event,
    )


def simulate_reduced_model(
    initial: InitialCondition
    = DEFAULT_INITIAL_CONDITION,
    t_span: tuple[float, float]
    = (0.0, 15.0),
    controller_function: ReducedController
    = reduced_model_controller,
    params: ReducedUnicycleParameters
    = DEFAULT_REDUCED_PARAMETERS,
    max_step: float = 0.005,
    output_step: float = 0.005,
) -> Any:
    """Simulate the reduced four-state model."""

    # Convert the physical lateral initial condition to:
    #
    # [theta, q2, theta_dot, u2]

    initial_state = reduced_state_from_physical(
        initial.theta,
        initial.r,
        initial.theta_dot,
        initial.r_dot,
        params,
    )

    def closed_loop_rhs(
        t: float,
        state: NDArray[np.float64],
    ) -> NDArray[np.float64]:

        lateral_force = controller_function(
            t,
            state,
            params,
        )

        return reduced_unicycle_model(
            t,
            state,
            lateral_force,
            params,
        )

    def theta_limit_event(
        t: float,
        state: NDArray[np.float64],
    ) -> float:

        del t

        theta = state[0]

        return (
            np.deg2rad(75.0)
            - abs(theta)
        )

    theta_limit_event.terminal = True
    theta_limit_event.direction = -1

    return solve_ivp(
        closed_loop_rhs,
        t_span,
        initial_state,
        method="RK45",
        t_eval=sample_times(
            t_span,
            output_step,
        ),
        rtol=1.0e-8,
        atol=1.0e-10,
        max_step=max_step,
        events=theta_limit_event,
    )


def full_lateral_history(
    solution: Any,
    params: UnicycleParameters
    = DEFAULT_PARAMETERS,
) -> NDArray[np.float64]:
    """Return [theta, r, theta_dot, r_dot]."""

    theta = solution.y[
        STATE_INDEX["theta"]
    ]

    r = solution.y[
        STATE_INDEX["r"]
    ]

    theta_dot = solution.y[
        STATE_INDEX["sigma1"]
    ]

    r_dot = (
        params.R * theta_dot
        + solution.y[
            STATE_INDEX["sigma_r"]
        ]
    )

    return np.vstack(
        [
            theta,
            r,
            theta_dot,
            r_dot,
        ]
    )


def reduced_lateral_history(
    solution: Any,
    params: ReducedUnicycleParameters
    = DEFAULT_REDUCED_PARAMETERS,
) -> NDArray[np.float64]:
    """Return [theta, r, theta_dot, r_dot]."""

    histories = [
        physical_lateral_state(
            state,
            params,
        )
        for state in solution.y.T
    ]

    return np.column_stack(
        histories
    )


def full_control_history(
    solution: Any,
    controller_function: FullController
    = full_model_controller,
    params: UnicycleParameters
    = DEFAULT_PARAMETERS,
) -> NDArray[np.float64]:
    """Return full-model force in the reduced/Appell F direction."""

    force = np.empty(
        solution.t.size,
        dtype=float,
    )

    for index, (time, state) in enumerate(
        zip(
            solution.t,
            solution.y.T,
        )
    ):
        control_input = np.asarray(
            controller_function(
                time,
                state,
                params,
            ),
            dtype=float,
        )

        # Full model uses:
        #
        # F1 = -F
        #
        # Therefore:
        #
        # F = -F1

        force[index] = (
            -control_input[0]
        )

    return force


def reduced_control_history(
    solution: Any,
    controller_function: ReducedController
    = reduced_model_controller,
    params: ReducedUnicycleParameters
    = DEFAULT_REDUCED_PARAMETERS,
) -> NDArray[np.float64]:
    """Return reduced-model lateral force F."""

    return np.array(
        [
            controller_function(
                time,
                state,
                params,
            )
            for time, state in zip(
                solution.t,
                solution.y.T,
            )
        ],
        dtype=float,
    )


def full_longitudinal_history(
    solution: Any,
) -> NDArray[np.float64]:
    """Return [gamma, phi] from the full model."""

    gamma = solution.y[
        STATE_INDEX["gamma"]
    ]

    phi = solution.y[
        STATE_INDEX["phi"]
    ]

    return np.vstack(
        [
            gamma,
            phi,
        ]
    )


def simulate_and_compare(
    initial: InitialCondition
    = DEFAULT_INITIAL_CONDITION,
    t_span: tuple[float, float]
    = (0.0, 15.0),
    full_params: UnicycleParameters
    = DEFAULT_PARAMETERS,
    reduced_params:
    ReducedUnicycleParameters | None = None,
) -> tuple[
    Any,
    Any,
    ReducedUnicycleParameters,
]:
    """Simulate both models with consistent lateral parameters."""

    if reduced_params is None:
        reduced_params = (
            ReducedUnicycleParameters
            .from_full_model(
                full_params
            )
        )

    full_solution = simulate_full_model(
        initial=initial,
        t_span=t_span,
        controller_function=full_model_controller,
        params=full_params,
    )

    reduced_solution = simulate_reduced_model(
        initial=initial,
        t_span=t_span,
        controller_function=reduced_model_controller,
        params=reduced_params,
    )

    return (
        full_solution,
        reduced_solution,
        reduced_params,
    )


def plot_model_comparison(
    full_solution: Any,
    reduced_solution: Any,
    full_params: UnicycleParameters
    = DEFAULT_PARAMETERS,
    reduced_params: ReducedUnicycleParameters
    = DEFAULT_REDUCED_PARAMETERS,
    full_controller_function: FullController
    = full_model_controller,
    reduced_controller_function: ReducedController
    = reduced_model_controller,
) -> tuple[
    plt.Figure,
    NDArray[plt.Axes],
]:
    """Plot lateral states and lateral force F."""

    full_output = full_lateral_history(
        full_solution,
        full_params,
    )

    reduced_output = reduced_lateral_history(
        reduced_solution,
        reduced_params,
    )

    full_force = full_control_history(
        full_solution,
        full_controller_function,
        full_params,
    )

    reduced_force = reduced_control_history(
        reduced_solution,
        reduced_controller_function,
        reduced_params,
    )

    figure, axes = plt.subplots(
        5,
        1,
        figsize=(6.8, 7.2),
        sharex=True,
    )

    labels = (
        r"$\theta$ (deg)",
        r"$r$ (m)",
        r"$\dot{\theta}$ (deg/s)",
        r"$\dot{r}$ (m/s)",
        r"$F$ (N)",
    )

    for index, axis in enumerate(
        axes[:4]
    ):
        full_values = full_output[index]
        reduced_values = reduced_output[index]

        if index in (0, 2):
            full_values = np.rad2deg(
                full_values
            )

            reduced_values = np.rad2deg(
                reduced_values
            )

        axis.plot(
            full_solution.t,
            full_values,
            linewidth=1.5,
            label="Full model",
        )

        axis.plot(
            reduced_solution.t,
            reduced_values,
            "--",
            linewidth=1.4,
            label="Reduced model",
        )

        axis.set_ylabel(
            labels[index]
        )

        axis.grid(
            True,
            alpha=0.35,
        )

    axes[4].plot(
        full_solution.t,
        full_force,
        linewidth=1.5,
        label="Full model",
    )

    axes[4].plot(
        reduced_solution.t,
        reduced_force,
        "--",
        linewidth=1.4,
        label="Reduced model",
    )

    axes[4].set_ylabel(
        labels[4]
    )

    axes[4].set_xlabel(
        "Time (s)"
    )

    axes[4].grid(
        True,
        alpha=0.35,
    )

    axes[0].legend(
        fontsize=8,
    )

    figure.tight_layout(
        pad=0.7,
    )

    return figure, axes


def plot_longitudinal_response(
    full_solution: Any,
) -> tuple[
    plt.Figure,
    NDArray[plt.Axes],
]:
    """Plot gamma and phi in a separate figure."""

    longitudinal_output = (
        full_longitudinal_history(
            full_solution
        )
    )

    gamma = np.rad2deg(
        longitudinal_output[0]
    )

    phi = longitudinal_output[1]

    figure, axes = plt.subplots(
        2,
        1,
        figsize=(6.8, 3.8),
        sharex=True,
    )

    axes[0].plot(
        full_solution.t,
        gamma,
        linewidth=1.5,
        color="tab:blue",
    )

    axes[0].set_ylabel(
        r"$\gamma$ (deg)"
    )

    axes[0].set_title(
        "Full-model longitudinal response"
    )

    axes[0].grid(
        True,
        alpha=0.35,
    )

    axes[1].plot(
        full_solution.t,
        phi,
        linewidth=1.5,
        color="tab:orange",
    )

    axes[1].set_ylabel(
        r"$\phi$ (rad)"
    )

    axes[1].set_xlabel(
        "Time (s)"
    )

    axes[1].grid(
        True,
        alpha=0.35,
    )

    figure.tight_layout(
        pad=0.7,
    )

    return figure, axes


def main() -> None:
    (
        full_solution,
        reduced_solution,
        reduced_params,
    ) = simulate_and_compare()

    print(
        "Full model: "
        f"success={full_solution.success}, "
        f"t_final={full_solution.t[-1]:.3f} s"
    )

    print(
        "Reduced model: "
        f"success={reduced_solution.success}, "
        f"t_final={reduced_solution.t[-1]:.3f} s"
    )

    full_force = full_control_history(
        full_solution,
    )

    reduced_force = reduced_control_history(
        reduced_solution,
        params=reduced_params,
    )

    print(
        "Full-model peak |F|: "
        f"{np.max(np.abs(full_force)):.6f} N"
    )

    print(
        "Reduced-model peak |F|: "
        f"{np.max(np.abs(reduced_force)):.6f} N"
    )

    plot_model_comparison(
        full_solution,
        reduced_solution,
        full_params=DEFAULT_PARAMETERS,
        reduced_params=reduced_params,
    )

    plot_longitudinal_response(
        full_solution,
    )

    plt.show()


if __name__ == "__main__":
    main()