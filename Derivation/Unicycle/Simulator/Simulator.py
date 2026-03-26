from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from Models import UnicycleModel
from Config import MODEL_T


class Simulator:
    def __init__(self, model, model2, params, compare=False):
        self.model: UnicycleModel = model
        self.model2: UnicycleModel = model2
        self.params = params
        self.sol = None
        self.state_labels = [
            r"$\psi$ (Yaw)",
            r"$\theta$ (Tilt)",
            r"$\phi$ (Roll)",
            "r (Linear Motor)",
            r"$\gamma$ (Pendulum)",
            r"$\dot{\psi}$",
            r"$\dot{\theta}$",
            r"$\dot{\phi}$",
            r"$\dot{r}$",
            r"$\dot{\gamma}$",
        ]
        self.compare = compare
        if compare:
            self.sol2 = None  # used to comparing two different simulations

    def _ode_wrapper(self, t, state, model: UnicycleModel, controller):
        u_input = controller(t, state)  # get control input from the controller function

        return model.get_equations_of_motion(t, state, u_input, self.params)

    def run(
        self,
        x0,
        t_span,
        t_eval,
        controller=None,
        x0_2=None,
        t_span_2=None,
        t_eval_2=None,
        controller_2=None,
    ):
        if controller is None:
            controller = lambda t, s: {"T_W": 0.0, "F_L": 0.0}

        sol = solve_ivp(
            fun=self._ode_wrapper,
            t_span=t_span,
            y0=x0,
            t_eval=t_eval,
            args=(self.model, controller),
            method="RK45",
            rtol=1e-6,
        )
        self.sol = sol
        if (
            self.compare
        ):  # solve again for the second simulation if we want to compare the results
            if x0_2 is None or t_span_2 is None or t_eval_2 is None:
                raise ValueError(
                    "For comparison, x0_2, t_span_2, and t_eval_2 must be provided."
                )
            if controller_2 is None:
                controller_2 = lambda t, s: {"T_W": 0.0, "F_L": 0.0}
            sol2 = solve_ivp(
                fun=self._ode_wrapper,
                t_span=t_span_2,
                y0=x0_2,
                t_eval=t_eval_2,
                args=(self.model2, controller_2),
                method="RK45",
                rtol=1e-6,
            )
            self.sol2 = sol2

    def plot(self):
        if self.sol is None:
            print("No data to plot. Run simulation first.")
            return

        fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        # (psi, theta, phi, gamma)
        for i in [0, 1, 2, 4]:
            axes[0].plot(self.sol.t, self.sol.y[i], label=self.state_labels[i])
        axes[0].set_ylabel("Angle (rad)")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title("System States Over Time")

        # (r)
        axes[1].plot(self.sol.t, self.sol.y[3], label=self.state_labels[3], color="red")
        axes[1].set_ylabel("Displacement (m)")
        axes[1].set_xlabel("Time (s)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def compare_plot(self):  # plot solution 1 and 2 together for comparison
        if self.sol is None or self.sol2 is None:
            print("Both simulations must be run before comparing.")
            return

        fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        # (psi, theta, phi, gamma)
        for i in [0, 1, 2, 4]:
            axes[0].plot(
                self.sol.t,
                self.sol.y[i],
                label=f"Simulation 1 - {self.state_labels[i]}",
            )
            axes[0].plot(
                self.sol2.t,
                self.sol2.y[i],
                "--",
                label=f"Simulation 2 - {self.state_labels[i]}",
            )
        axes[0].set_ylabel("Angle (rad)")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title("Comparison of System States Over Time")

        # (r)
        axes[1].plot(
            self.sol.t,
            self.sol.y[3],
            label=f"Simulation 1 - {self.state_labels[3]}",
            color="red",
        )
        axes[1].plot(
            self.sol2.t,
            self.sol2.y[3],
            "--",
            label=f"Simulation 2 - {self.state_labels[3]}",
            color="orange",
        )
        axes[1].set_ylabel("Displacement (m)")
        axes[1].set_xlabel("Time (s)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()
