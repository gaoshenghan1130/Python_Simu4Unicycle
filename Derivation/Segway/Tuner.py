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

if __name__ == "__main__":
 

    def simulate(result):
        run_simulation(
            initial_state=[0.0, 0.0, 0.0, 0.0],
            time_span=(0, 15),
            control_mode=CONTROL_MODE.VELOCITY,
            controller_type=CONTROL_STRATEGY.PD,
            desired_gamma=0.0,
            desired_velocity=0.5,
            desired_position=0.0,
            model_type=MODEL_T.ROLLING_RESISTANCE,
            record_Torque=False,
            export_CSV_flag=False,
            plot_results=False,
            export2list=result
        )

    mode_segments = parse_mixed_csv("c:\\Users\\labuser\\Desktop\\unicycle_project_umich\\LOGS\\ValidLogs\\log_2026-04-01_14-20-46.533.csv")
    data = None
    for item in mode_segments:
        if item['mode'] == 'Velocity':
            data = item['data']
            break

    realdata = getVelocityFromData(data)

    def penaltyFunc(resultStorage):
        error = 0.0

        if realdata is None or len(realdata) == 0:
            raise ValueError("Real data is empty, cannot compute penalty.")
        if resultStorage is None or len(resultStorage) == 0:
            raise ValueError("Simulation result is empty, cannot compute penalty.")
        
        numeric_results = [item for item in resultStorage if isinstance(item[0], (int, float))]

        for item in realdata:
            time = item[0]
            real_vel = item[1]
            closest_time = min(numeric_results, key=lambda x: abs(x[0] - time))
            sim_vel = closest_time[1]
            # calculate the squared error
            error += (real_vel - sim_vel) ** 2
        
        print(f"Current penalty (MSE): {error / len(realdata)}")
        return error/ len(realdata)



    best_params = tune_parameters(paramDict = {"Kp": 1.0, "Kd": 0.1},
                                    simulation=simulate,
                                    realdata=realdata,
                                    ax_real_name="Real Velocity Data",
                                    ax_try_name="Simulated Velocity Data",
                                    penaltyFunc=penaltyFunc
                                    )
    print("Best parameters found:", best_params)