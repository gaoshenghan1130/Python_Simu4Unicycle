from ParamTuners import (
    ParamTuner,
    tuningPlotter,
    parse_mixed_csv,
    getPositionFromData,
    getVelocityFromData,
)
from Runner import run_simulation
from Config import MODEL_T, CONTROL_STRATEGY, CONTROL_MODE


def tune_parameters(
    paramDict,
    simulation,
    realdata,
    columnIndex=1,
    penaltyFunc=None,
    max_iterations=100,
    ax_real_name="real data",
    ax_try_name="try data",
    export_csv=False,
    csv_filename="tuning_results.csv",
):
    """Tune the parameters of the controller using the paramTuner class."""
    simluation_result_in_each_iteration = []

    tuner: ParamTuner = ParamTuner(paramDict, simulation, penaltyFunc)
    plotter: tuningPlotter = tuningPlotter(
        topic="Parameter Tuning Progress",
        ax_real_name=ax_real_name,
        ax_try_name=ax_try_name,
        ax_real_set=realdata,
        columnIndex=columnIndex,
    )
    csvfilename = "Position_" + csv_filename if columnIndex == 1 else "Velocity_" + csv_filename # 1 for position, 2 for velocity


    plotter.start()
    best_params = tuner.tune(
        resultStorage=simluation_result_in_each_iteration,
        max_iter=max_iterations,
        plotter=plotter,
        export_csv=export_csv,
        csv_filename=csvfilename,
    )
    return best_params
