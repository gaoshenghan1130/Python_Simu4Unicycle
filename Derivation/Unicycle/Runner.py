from Models import UnicycleModel
from Simulator import Simulator
from Parameters import params
from Config import MODEL_T
import numpy as np


def _prepare_model(model: UnicycleModel, recalculate: bool):
    """Handle model loading logic"""
    if recalculate or not model.isLoaded():
        model.load_dynamics(recalculate=True) 
    else:
        model.load_dynamics(recalculate=False)


def run_simulation(
    initial_state,
    t_span,
    model_id: MODEL_T = MODEL_T.KANE,
    controller=None,
    compare=False,
    model2_id: MODEL_T = MODEL_T.KANE,
    controller_2=None,
    recalculate=False,
    t_eval=None,
):
    if controller is None:
        controller = lambda t, s: {"T_W": 0.0, "F_L": 0.0}
    if controller_2 is None:
        controller_2 = controller
    if t_eval is None:
        t_eval = np.linspace(t_span[0], t_span[1], 500)

    model = UnicycleModel(model_id)
    model2 = UnicycleModel(model2_id)

    params_dict = {
        "R": params.R,
        "h": params.h,
        "m": params.m,
        "m_w": params.m_w,
        "m_l": params.m_l,
        "g": params.g,
    }

    _prepare_model(model, recalculate)
    if compare:
        _prepare_model(model2, recalculate)

    sim = Simulator(model, model2, params_dict, compare=compare)

    if not compare:
        sim.run(x0=initial_state, t_span=t_span, t_eval=t_eval, controller=controller)
        sim.plot()
    else:
        sim.run(
            x0=initial_state,
            t_span=t_span,
            t_eval=t_eval,
            controller=controller,
            x0_2=initial_state,
            t_span_2=t_span,
            t_eval_2=t_eval,
            controller_2=controller_2,
        )
        sim.compare_plot()
