from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from Models import UnicycleModel

class Simulator:
    def __init__(self, model, params):
        self.model : UnicycleModel= model
        self.params = params
        self.sol = None
        self.state_labels = [
            r'$\psi$ (Yaw)', r'$\theta$ (Tilt)', r'$\phi$ (Roll)', 
            'r (Linear Motor)', r'$\gamma$ (Pendulum)',
            r'$\dot{\psi}$', r'$\dot{\theta}$', r'$\dot{\phi}$', 
            r'$\dot{r}$', r'$\dot{\gamma}$'
        ]

    def _ode_wrapper(self, t, state, controller_func):
        # 1. Get control inputs from the controller function
        u_input = controller_func(t, state)
        
        return self.model.get_equations_of_motion(t, state, u_input, self.params)

    def run(self, x0, t_span, t_eval, controller=None):
        if controller is None:
            controller = lambda t, s: {'T_W': 0.0, 'F_L': 0.0}

        sol = solve_ivp(
            fun=self._ode_wrapper,
            t_span=t_span,
            y0=x0,
            t_eval=t_eval,
            args=(controller,),
            method='RK45',
            rtol=1e-6
        )
        self.sol = sol
        return sol
    
    def plot(self):
        if self.sol is None:
            print("No data to plot. Run simulation first.")
            return

        fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        # (psi, theta, phi, gamma)
        for i in [0, 1, 2, 4]:
            axes[0].plot(self.sol.t, self.sol.y[i], label=self.state_labels[i])
        axes[0].set_ylabel('Angle (rad)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title('System States Over Time')

        # (r)
        axes[1].plot(self.sol.t, self.sol.y[3], label=self.state_labels[3], color='red')
        axes[1].set_ylabel('Displacement (m)')
        axes[1].set_xlabel('Time (s)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()