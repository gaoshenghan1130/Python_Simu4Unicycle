from Models import UnicycleModel
from Simulator import Simulator
from Parameters import params

def run_simulation(initial_state, t_span, controller=None):
    model : UnicycleModel = UnicycleModel()

    params_dict = {
        'R': params.R,
        'h': params.h,
        'm': params.m,
        'm_w': params.m_w,
        'm_l': params.m_l,
        'g': params.g
    }
    
    sim = Simulator(model, params_dict)
    sim.run(initial_state, t_span, controller)
    sim.plot()