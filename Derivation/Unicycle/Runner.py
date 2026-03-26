from Models import UnicycleModel
from Simulator import Simulator
from Parameters import params
from Config import MODEL_T

def run_simulation(initial_state, t_span, model_id : MODEL_T = MODEL_T.KANE, controller=None, compare=False, model2_id : MODEL_T = MODEL_T.KANE, controller_2=None):

    # First generate Model 
    model : UnicycleModel = UnicycleModel(model_id)
    model2 : UnicycleModel = UnicycleModel(model2_id)

    params_dict = {
        'R': params.R,
        'h': params.h,
        'm': params.m,
        'm_w': params.m_w,
        'm_l': params.m_l,
        'g': params.g
    }

    if not compare:
        sim = Simulator(model, model2, params_dict) # model2 won't be used here if compare is False
        sim.run(initial_state, t_span, controller)
        sim.plot()

    else:
        initial_state_2 = initial_state
        t_span_2 = t_span
        sim = Simulator(model, model2, params_dict, compare=True)
        sim.run(initial_state, t_span, controller, initial_state_2, t_span_2, controller_2)
        sim.compare_plot()