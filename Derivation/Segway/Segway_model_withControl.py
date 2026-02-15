from Segway_model2 import Model2
import numpy as np
from Parameters import params
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from enum import Enum, auto

class Mode(Enum):   
    BALANCE = auto()
    VELOCITY= auto()
    POSITION = auto()

class ModelWithControl(Model2):
    def __init__(self):
        super().__init__()

        # The system model is in the form of z' = A z + B M, where M is the magnitude of the control input (torque applied to the wheel)
        # Here we should define the change of M based on the desired velocity and the current velocity, which is a simple proportional control law. We can later tune the gain to achieve better performance.
        self.control = self.control_law

    def control_law(self, z, mode, desired_gamma=0.0, desired_velocity=0.0, desired_position=0.0):
        # Z is the state vector [x_c, x_c', gamma, gamma']
        z = np.asarray(z)
        x_c = z[0]
        x_c_dot = z[1]
        gamma = z[2]
        gamma_dot = z[3]
        K_gamma = params.K_gamma
        K_dgamma = params.K_dgamma
        K_velocity = params.K_velocity
        K_position = params.K_position

        if mode == Mode.BALANCE:
            return +K_gamma * (gamma - desired_gamma) + K_dgamma * gamma_dot
        elif mode == Mode.VELOCITY:
            return +K_gamma * (gamma - desired_gamma) + K_dgamma * gamma_dot + K_velocity * (x_c_dot - desired_velocity)
        elif mode == Mode.POSITION:
            return +K_gamma * (gamma - desired_gamma) + K_dgamma * gamma_dot + K_velocity * (x_c_dot - desired_velocity) + K_position * (x_c - desired_position)
        else:
            raise ValueError("Invalid control mode")
        
    def close_loop_dynamics(self, z, mode, desired_gamma=0.0, desired_velocity=0.0, desired_position=0.0):
        M = self.control(z, mode, desired_gamma, desired_velocity, desired_position)
        return self.state_space(z, M)
    
    def simulate(self, z0, t_span, mode, desired_gamma=0.0, desired_velocity=0.0, desired_position=0.0):
        sol = solve_ivp(lambda t, z: self.close_loop_dynamics(z, mode, desired_gamma, desired_velocity, desired_position), t_span, z0, t_eval=np.linspace(t_span[0], t_span[1], 1000), method='RK45')

        if mode == Mode.BALANCE or mode == Mode.POSITION:
            plt.plot(sol.t, sol.y[0], label='Position (x_c)')
        if mode == Mode.VELOCITY:
            plt.plot(sol.t, sol.y[1], label='Velocity (x_c_dot)')
        plt.plot(sol.t, sol.y[2], label='Tilt angle (gamma)')
        plt.xlabel('Time (s)')
        plt.legend()
        plt.title(f'Segway Simulation - Mode: {mode.name}')
        plt.grid()
        plt.show()
        return sol.t, sol.y
            
    
Segway_model_withControl = ModelWithControl()  # Light singleton pattern


    

    

        


        