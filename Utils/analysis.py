from Phaser.phaseRawData import parse_mixed_csv, phaseMathData, phaseMojucoData
from Plotters.multiploter import plot_comparison

raw_data_path = r"c:\Users\labuser\Desktop\unicycle_project_umich\LOGS\ValidLogs\log_2026-04-01_14-20-46.533.csv"

math_position_data_path = r"c:\Users\labuser\Desktop\Matlab_Simu4Unicycle\Derivation\Segway\LOG\simulation_POSITION_Model_motorDamp_Controller.csv"
math_velocity_data_path = r"c:\Users\labuser\Desktop\Matlab_Simu4Unicycle\Derivation\Segway\LOG\simulation_VELOCITY_Model_motorDamp_Controller.csv"

mojuco_position_data_path = r"c:\Users\labuser\Desktop\Unicycle_Mujoco_Simulation\csv\mydata\position.csv"
mojuco_velocity_data_path = r"c:\Users\labuser\Desktop\Unicycle_Mujoco_Simulation\csv\mydata\velocity.csv"


if __name__ == "__main__":
    real_parsed_data = parse_mixed_csv(raw_data_path)
    # use the last postion mode and velocity mode data for comparison
    real_position_data = None
    real_velocity_data = None
    for data in real_parsed_data:
        if data['mode'] == "position":
            real_position_data =data['data']
        elif data['mode'] == "velocity":
            real_velocity_data = data['data']


    mathPositionData = phaseMathData(math_position_data_path)
    mathVelocityData = phaseMathData(math_velocity_data_path)
    mojucoPositionData = phaseMojucoData(mojuco_position_data_path)
    mojucoVelocityData = phaseMojucoData(mojuco_velocity_data_path)


    all_position_data = {
        "Real": real_position_data,
        "Math": mathPositionData,
        "Mujoco": mojucoPositionData
    }
    all_velocity_data = {
        "Real": real_velocity_data,
        "Math": mathVelocityData,
        "Mujoco": mojucoVelocityData
    }

    if all_position_data['Real'] is not None and all_position_data['Math'] is not None and all_position_data['Mujoco'] is not None:
        plot_comparison(all_position_data, output_file='position_comparison.png')
    if all_velocity_data['Real'] is not None and all_velocity_data['Math'] is not None and all_velocity_data['Mujoco'] is not None:
        plot_comparison(all_velocity_data, output_file='velocity_comparison.png', isvelocity= True)