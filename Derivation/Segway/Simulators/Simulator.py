import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from Config import MODEL_T, CONTROL_MODE, CONTROL_STRATEGY
from Models import Model
from Controllers import Controller


class Simulator:
    def __init__(self, model: Model, controller : Controller):
        self.model = model
        self.controller = controller
        self.sol = None

    def close_loop_dynamics(self, t, z, desired_gamma, desired_velocity, desired_position):
        M = self.controller.control_law(z, desired_gamma, desired_velocity, desired_position) # Get torque
        dzdt = self.model.state_space(z, M,) # Get state derivatives
        return dzdt

    def simulate(self, z0, t_span, t_eval, desired_gamma, desired_velocity, desired_position):
        sol = solve_ivp(lambda t, z: self.close_loop_dynamics(t, z, desired_gamma, desired_velocity, desired_position), t_span, z0, t_eval=t_eval)
        self.sol = sol

    def plot_results(self):
        if self.sol is None:
            raise ValueError("No simulation results to plot. Please run simulate() first.")
        
        plt.figure(figsize=(12, 8))
        plt.subplot(2, 1, 1)
        plt.plot(self.sol.t, self.sol.y[0], label='Position (x_c)')
        plt.plot(self.sol.t, self.sol.y[2], label='Angle (gamma)')
        plt.title('Segway Simulation Results')
        plt.xlabel('Time (s)')
        plt.ylabel('State')
        plt.legend()
        plt.grid()

        plt.subplot(2, 1, 2)
        plt.plot(self.sol.t, self.sol.y[1], label='Velocity (x_c_dot)')
        plt.plot(self.sol.t, self.sol.y[3], label='Angular Velocity (gamma_dot)')
        plt.xlabel('Time (s)')
        plt.ylabel('State Derivative')
        plt.legend()
        plt.grid()

        plt.tight_layout()
        plt.show()

    @staticmethod
    def create_simulator(model_type: MODEL_T, control_strategy: CONTROL_STRATEGY, control_mode: CONTROL_MODE)-> "Simulator":
        # create model and controller instances using the factory
        from Factories import Model_Factory, Controller_Factory
        model = Model_Factory.create_model(model_type)
        controller = Controller_Factory.create_controller(control_strategy, control_mode)
        
        # create simulator instance
        return Simulator(model, controller)