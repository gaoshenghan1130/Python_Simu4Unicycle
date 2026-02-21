from Simulators import Simulator
from Factories import Model_Factory, Controller_Factory
from Config import MODEL_T, CONTROL_STRATEGY, CONTROL_MODE
import numpy as np


def run_simulation(
    initial_state,
    time_span,
    control_mode: CONTROL_MODE = CONTROL_MODE.BALANCE,
    desired_gamma=0.0,
    desired_velocity=0.0,
    desired_position=0.0,
    model_type: MODEL_T = MODEL_T.LINEAR,
    controller_type: CONTROL_STRATEGY = CONTROL_STRATEGY.PD,
    plot_results=True,
    record_Torque=False
):
    # Create simulator instance using the factory method
    simulator = Simulator.create_simulator(model_type, controller_type, control_mode, record_Torque=record_Torque)

    # Simulate the system
    t_eval = np.linspace(time_span[0], time_span[1], 1000)
    simulator.simulate(initial_state, time_span, t_eval, desired_gamma, desired_velocity, desired_position)
    if plot_results:
        simulator.plot_results()


def modelCheck(model_type: MODEL_T, initial_state, time_span):
    model = Model_Factory.create_model(model_type)
    model.Check()
