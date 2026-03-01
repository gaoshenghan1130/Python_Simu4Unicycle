import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from Config import MODEL_T, CONTROL_MODE, CONTROL_STRATEGY
from Models import Model
from Controllers import Controller


class Simulator:
    def __init__(
        self,
        model: Model,
        controller: Controller,
        record_Torque=False,
    ):
        self.model = model
        self.controller = controller
        self.sol = None
        self.record_Torque = record_Torque
        self.torque_history = []

    def close_loop_dynamics(
        self, t, z, desired_gamma, desired_velocity, desired_position
    ):
        M = self.controller.control_law(
            z=z,
            t=t,
            desired_gamma=desired_gamma,
            desired_velocity=desired_velocity,
            desired_position=desired_position,
        )  # Get torque
        dzdt = self.model.state_space(z, M)  # Get state derivatives
        if self.record_Torque:
            self.torque_history.append((t, M))
        return dzdt

    def simulate(
        self, z0, t_span, t_eval, desired_gamma, desired_velocity, desired_position
    ):
        sol = solve_ivp(
            lambda t, z: self.close_loop_dynamics(
                z=z,
                t=t,
                desired_gamma=desired_gamma,
                desired_velocity=desired_velocity,
                desired_position=desired_position,
            ),
            t_span,
            z0,
            t_eval=t_eval,
            method="RK45",
        )
        self.sol = sol

    def plot_results(self):
        if self.sol is None:
            raise ValueError(
                "No simulation results to plot. Please run simulate() first."
            )

        plt.figure(figsize=(12, 8))
        plt.subplot(2, 1, 1)

        ax1 = plt.gca()
        ax2 = ax1.twinx()

        # Position (left axis)
        ax1.plot(self.sol.t, self.sol.y[0], label="Position (x_c)")
        ax1.set_ylabel("Position (m)")
        ax1.set_xlabel("Time (s)")
        ax1.grid()

        # Convert rad → deg
        gamma_deg = np.degrees(self.sol.y[2])

        # Angle (right axis)
        ax2.plot(self.sol.t, gamma_deg, linestyle="--", label="Angle (gamma)")
        ax2.set_ylabel("Angle (deg)")

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2)

        plt.title("Segway Simulation Results")
        plt.xlabel("Time (s)")
        plt.ylabel("State")
        plt.legend()
        plt.grid()

        plt.subplot(2, 1, 2)
        plt.plot(self.sol.t, self.sol.y[1], label="Velocity (x_c_dot)")
        plt.plot(self.sol.t, self.sol.y[3], label="Angular Velocity (gamma_dot)")
        plt.xlabel("Time (s)")
        plt.ylabel("State Derivative")
        plt.legend()
        plt.grid()

        if self.record_Torque:
            plt.figure(figsize=(12, 4))
            torque_times, torques = zip(*self.torque_history)
            plt.plot(torque_times, torques, label="Control Torque (M)")
            plt.title("Control Torque Over Time")
            plt.xlabel("Time (s)")
            plt.ylabel("Torque (N*m)")
            plt.legend()
            plt.grid()

        plt.tight_layout()
        plt.show()

    def export_CSV(self):
        if self.sol is None:
            raise ValueError(
                "No simulation results to export. Please run simulate() first."
            )

        import os

        os.makedirs("LOG", exist_ok=True)
        filename = f"LOG/simulation_{self.controller.control_mode.name}_{self.model.__class__.__name__}_{self.controller.__class__.__name__}.csv"

        data = np.vstack((self.sol.t, self.sol.y)).T
        header = "time,x_c,x_c_dot,gamma,gamma_dot"
        np.savetxt(filename, data, delimiter=",", header=header, comments="")

    @staticmethod
    def create_simulator(
        model_type: MODEL_T,
        control_strategy: CONTROL_STRATEGY,
        control_mode: CONTROL_MODE,
        record_Torque=False,
    ) -> "Simulator":
        # create model and controller instances using the factory
        from Factories import Model_Factory, Controller_Factory

        model = Model_Factory.create_model(model_type)
        controller = Controller_Factory.create_controller(
            control_strategy, control_mode
        )

        # create simulator instance
        return Simulator(model, controller, record_Torque=record_Torque)
