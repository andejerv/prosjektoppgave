import pandas as pd
import numpy as np
import os
from hydra import initialize, compose
from omegaconf import DictConfig, OmegaConf
import yaml
from docking_algorithms.utils.types import OptimalTrajectories
import pickle 
from docking_algorithms.utils.inf2pipi import inf2pipi

class DataLogger:
    def __init__(self, filename: str, base_directory: str, controller_type: str,
                 batch: str, run_id: int, quay_type: str, run_description: str) -> None:
        """
        Initializes the DataLogger with the specified filename and directory.
        
        :param filename: str - The name of the file to log data to.
        :param directory: str - The directory where the file will be saved.
        """

        self.controller_type = controller_type
        self.batch = batch
        self.run_id = run_id
        self.quay_type = quay_type

        if run_description == '':
            self.directory = os.path.join(base_directory, controller_type, batch, f"run{str(run_id)}")
        else:
            self.directory = os.path.join(base_directory, controller_type, batch, f"run{str(run_id)}", run_description)

        os.makedirs(self.directory, exist_ok=True)
        # Check if the directory exists, and raise an error if it doesn't
        # if not os.path.exists(self.directory):
        #     raise FileNotFoundError(f"The directory '{self.directory}' does not exist. Please create it manually.")


        # Create the full file path
        self.filepath = os.path.join(self.directory, filename)
        columns = ['t', 'eta', 'eta_dot', 'eta_d', 'eta_d_dot', 'chi_d', 'chi', 'nu', 'nu_d', 
                    'accel_body', 'control_forces_body', 'target_wp',
                    'side_normal', 'side_line_point', 'front_normal', 'front_line_point', 't_switch']

        # Check if the file already exists, and raise an error if it does
        if os.path.exists(self.filepath):
            raise FileExistsError(f"The file '{self.filepath}' already exists. Terminating to avoid overwriting the file.")

        # If the file does not exist, create it with a header
        with open(self.filepath, 'x', newline='') as f:
            f.write(f"controller type: {controller_type}, batch: {batch}, run id: {run_id}, quay type: {quay_type}\n")
            # Create an empty DataFrame with the desired columns
            df = pd.DataFrame(columns=columns)
            df.to_csv(f, header=columns, index=False)

    
    def log_data(self, t=None, eta=None, eta_dot=None, eta_d=None, eta_d_dot=None,
                 chi_d=None, chi=None, nu=None, nu_d=None, accel_body=None, 
                 control_forces_body=None, target_wp=None, t_switch=None,
                 side_normal=None, side_line_point=None,
                 front_normal=None, front_line_point=None) -> None:
        """
        Logs a new data entry to the CSV file.
        
        :param t: float - The time variable.
        :param eta: np.ndarray - The eta variable (state).
        :param eta_dot: np.ndarray - The derivative of eta.
        :param eta_d: np.ndarray - The desired state variable.
        :param eta_d_dot: np.ndarray - The derivative of the desired state.
        :param accel_body: np.ndarray - The acceleration in the body frame.
        :param control_forces_body: np.ndarray - The control forces in the body frame.
        :param target_wp: np.ndarray - The target waypoints.
        """
        # nan = np.array([np.nan, np.nan, np.nan])
        nan = np.nan

        # Create a DataFrame from the new data
        data = {
            't': t if t is not None else np.nan,
            'eta': ','.join(map(str, eta)) if eta is not None else nan,
            'eta_dot': ','.join(map(str, eta_dot)) if eta_dot is not None else nan,
            'eta_d': ','.join(map(str, eta_d)) if eta_d is not None else nan,
            'eta_d_dot': ','.join(map(str, eta_d_dot)) if eta_d_dot is not None else nan,
            'chi_d': chi_d if chi_d is not None else np.nan,
            'chi': chi if chi is not None else np.nan,
            'nu': ','.join(map(str, nu)) if nu is not None else nan,
            'nu_d': ','.join(map(str, nu_d)) if nu_d is not None else nan,
            'accel_body': ','.join(map(str, accel_body)) if accel_body is not None else nan,
            'control_forces_body': ','.join(map(str, control_forces_body)) if control_forces_body is not None else nan,
            'target_wp': ','.join(map(str, target_wp)) if target_wp is not None else nan,
            'side_normal': ','.join(map(str, side_normal)) if side_normal is not None else nan,
            'side_line_point': ','.join(map(str, side_line_point)) if side_line_point is not None else nan,
            'front_normal': ','.join(map(str, front_normal)) if front_normal is not None else nan,
            'front_line_point': ','.join(map(str, front_line_point)) if front_line_point is not None else nan,
            't_switch': t_switch if t_switch is not None else np.nan,    # times when switching from one phase to another
        }
        # Convert to DataFrame
        data_df = pd.DataFrame(data, index=[0])  # Ensure index is [0] to create a single row

        # Append the new data to the CSV file
        data_df.to_csv(self.filepath, mode='a', header=False, index=False)    

 
    def save_configs_to_file(self, configs: list) -> None:
        output_folder = self.directory
        # Convert output_folder to a string if it's a DictConfig
        if isinstance(output_folder, dict) or isinstance(output_folder, OmegaConf):
            output_folder = OmegaConf.to_container(output_folder, resolve=True)
            output_folder = output_folder["directory"] if "directory" in output_folder else output_folder

        # Generate the full output file path
        output_file = os.path.join(output_folder, f"config.yaml")

        # Ensure the output folder exists
        os.makedirs(output_folder, exist_ok=True)
        
        try:
            with open(output_file, 'w') as file:
                file.write(f"controller type: {self.controller_type}, batch: {self.batch}, run id: {self.run_id}, quay type: {self.quay_type}\n\n")
                # Write configurations to file
                for i, cfg in enumerate(configs):
                    yaml.safe_dump(
                        OmegaConf.to_container(cfg, resolve=True),
                        file,
                        default_flow_style=False,
                        sort_keys=False
                    )
                    # Add a blank line
                    file.write("\n")
            print(f"Configurations saved to: {output_file}")
        except Exception as e:
            print(f"Error occurred: {e}")
        return
    

    def save_mpc_trajectories(self, trajectory: OptimalTrajectories):
        """
        Appends a new trajectory instance to the existing pickle file.

        The heading angles are relative north, in hte range (-pi, pi])
        """
        filename = "mpc_opt_trajectories.pkl"
        filepath = os.path.join(self.directory, "mpc_output", filename)
        
        # Ensure the directory exists
        # os.makedirs(self.directory, exist_ok=True)
        output_dir = os.path.dirname(filepath)
        os.makedirs(output_dir, exist_ok=True)


        try:
            # Try loading existing data
            with open(filepath, "rb") as f:
                data = pickle.load(f)
        except (FileNotFoundError, EOFError):  
            # File not found or empty → Create new list
            # print("Pickle file not found or empty, creating a new one.")
            data = []

        # Append new trajectory and save
        data.append(trajectory)

        with open(filepath, "wb") as f:
            pickle.dump(data, f)


def load_mpc_trajectories(directory: str):
    """Loads all stored trajectory objects from a single pickle file."""
    filename = "mpc_opt_trajectories.pkl"
    filepath = os.path.join(directory, filename)
    try:
        with open(filepath, "rb") as f:
            mpc_data = pickle.load(f)
            return mpc_data
    except FileNotFoundError:
        print(f'File not found')
        return
    

def load_config(cfg_name: str, cfg_path: str="../config") -> DictConfig:
    with initialize(config_path=cfg_path, version_base=None):  # Base directory for YAML files
        cfg = compose(config_name=cfg_name)  # Load the base configuration file
    return cfg
