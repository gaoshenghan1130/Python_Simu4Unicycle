import os
from dotenv import load_dotenv

from Phaser.phaseRawData import parse_mixed_csv, phaseMathData, phaseMojucoData
from Plotters.multiploter import plot_comparison

load_dotenv()

raw_data_path = os.getenv("RAW_DATA_PATH")
math_position_data_path = os.getenv("MATH_POS_PATH")
math_velocity_data_path = os.getenv("MATH_VEL_PATH")
mojuco_position_data_path = os.getenv("MUJOCO_POS_PATH")
mojuco_velocity_data_path = os.getenv("MUJOCO_VEL_PATH")

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