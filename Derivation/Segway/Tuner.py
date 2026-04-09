from ParamTuners import ParamTuner, tuningPlotter, parse_mixed_csv, getPositionFromData, getVelocityFromData
from Runner import run_simulation
from Config import MODEL_T, CONTROL_STRATEGY, CONTROL_MODE

def tune_parameters(paramDict, simulation, realdata, columnIndex = 1, penaltyFunc = None, max_iterations=100, ax_real_name = "real data", ax_try_name = "try data"):
    """Tune the parameters of the controller using the paramTuner class."""
    simluation_result_in_each_iteration = []

    tuner : ParamTuner = ParamTuner(paramDict, simulation, penaltyFunc)
    plotter: tuningPlotter = tuningPlotter(topic= "Parameter Tuning Progress",
                                          ax_real_name=ax_real_name,
                                          ax_try_name=ax_try_name,
                                          ax_real_set=realdata,
                                          columnIndex = columnIndex)
    plotter.start()
    best_params = tuner.tune(resultStorage= simluation_result_in_each_iteration,
                              max_iter=max_iterations,
                              plotter=plotter   
                              )
    return best_params
