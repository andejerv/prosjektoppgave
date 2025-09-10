from dataclasses import dataclass
from typing import Any, List, Dict
import numpy as np
import os
import pandas as pd
import matplotlib.pyplot as plt
import plotly.io as pio
import csv
from generate_results.types import ProcessedData, WindData
from generate_results.types import Statistics, Metric, MetricCollection, StatisticsCollection


def read_and_process_logged_data(filepath: str) -> ProcessedData:
    """
    Reads the data from the specified CSV file and processes it into a structured format
    for easier computations and plotting.
    """
    validate_file(filepath=filepath)

    # Read the CSV file without reading the header
    df = pd.read_csv(filepath, skiprows=1)

    # Process numeric array-like fields
    def parse_array_column(column):
        """Parses a column containing comma-separated array strings into a NumPy array."""
        return np.array([np.fromstring(entry, sep=',') if isinstance(entry, str) else np.array([np.nan, np.nan, np.nan]) for entry in column])

    return ProcessedData(
        t=df['t'].to_numpy(),
        eta=parse_array_column(df['eta']),
        eta_dot=parse_array_column(df['eta_dot']),
        eta_d=parse_array_column(df['eta_d']),
        eta_d_dot=parse_array_column(df['eta_d_dot']),
        chi_d=df['chi_d'].to_numpy(),
        chi=df['chi'].to_numpy(),
        nu=parse_array_column(df['nu']),
        nu_d=parse_array_column(df['nu_d']),
        accel_body=parse_array_column(df['accel_body']),
        control_forces_body=parse_array_column(df['control_forces_body']),
        target_wp=parse_array_column(df['target_wp']),
        t_switch=df['t_switch'].dropna().to_numpy()
    )

def read_and_process_wind_data(filepath: str) -> WindData:
    validate_file(filepath=filepath)

    df = pd.read_csv(filepath, header=None, skiprows=1)  # Skip the header row if it exists

    return WindData(
        t=df.iloc[:, 0].to_numpy(),  # Time
        X=df.iloc[:, 1].to_numpy(),  # Surge force
        Y=df.iloc[:, 2].to_numpy(),  # Sway force
        N=df.iloc[:, 3].to_numpy(),  # Yaw moment
    )

def read_and_process_statistics(filepath: str,
                                statistics_collection: StatisticsCollection) -> StatisticsCollection:
    
    validate_file(filepath=filepath)

    # Read the first line to extract controller type and batch
    with open(filepath, 'r') as file:
        first_line = file.readline().strip()
    parts = first_line.split(",")
    
    # Parse controller type and batch from the first line
    controller_type = parts[0].split(":")[1].strip()
    batch = parts[1].split(":")[1].strip()

    # Read the rest of the CSV (skip the first line with metadata)
    df = pd.read_csv(filepath, skiprows=1)

    # Iterate through each row in the CSV and construct Statistics objects
    for _, row in df.iterrows():
        metric_type = row['Metric type'].strip()
        mean = float(row['Mean'])
        std_dev = float(row['Std Dev'])

        # Create the Statistics object
        stat = Statistics(metric_type=metric_type, mu=mean, sigma=std_dev)

        # Update the StatisticsCollection
        if controller_type not in statistics_collection.statistic_collection:
            statistics_collection.statistic_collection[controller_type] = {}

        if batch not in statistics_collection.statistic_collection[controller_type]:
            statistics_collection.statistic_collection[controller_type][batch] = {}

        # Add the metric's statistics
        statistics_collection.statistic_collection[controller_type][batch][metric_type] = stat

    return statistics_collection


def save_fig(output_dir: str, fig_name: str, file_format: str = 'svg') -> None:
    """
    Saves the figure to the specified directory and filename with the given file format.
    
    :param output_dir: str - Directory where the figure will be saved.
    :param fig_name: str - Name of the saved figure file.
    :param file_format: str - File format (default is 'png'). Can be 'png', 'pdf', 'svg', etc.
    """
    # Ensure the directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Construct the full file path
    file_path = os.path.join(output_dir, f"{fig_name}.{file_format}")

    # Save the figure
    plt.savefig(file_path, format=file_format,  bbox_inches='tight')
    print(f"Figure saved to {file_path}")

def save_fig_plotly(output_dir: str, fig_name: str, file_format: str, fig) -> None:
    """
    Saves the figure to the specified directory and filename with the given file format.
    
    :param output_dir: str - Directory where the figure will be saved.
    :param fig_name: str - Name of the saved figure file.
    :param file_format: str - File format (default is 'png'). Can be 'png', 'pdf', 'svg', etc.
    """
    # Ensure the directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Construct the full file path
    file_path = os.path.join(output_dir, f"{fig_name}.{file_format}")

    # Save the figure
    fig.write_image(file_path)
    print(f"Figure saved to {file_path}")


def save_metrics_single_run(output_dir: str, metrics_list: List[Metric],
                 controller_type: str, batch: str, run_id: int, quay_type: str) -> None:
    filename = "metrics.csv"
    filepath = os.path.join(output_dir, filename)

    # Open the file in write mode ('w'), or append mode ('a') if needed
    with open(filepath, mode='w', newline='') as file:
        writer = csv.writer(file)
        file.write(f"controller type: {controller_type}, batch: {batch}, run id: {run_id}, quay type: {quay_type}\n")

        # Write the header (column names)
        writer.writerow(["Metric type", "Value"])

        # Write each metric in the list
        for m in metrics_list:
            writer.writerow([m.metric_type, m.value])

    print(f"Metrics saved to {filepath}")


def save_statistics_single_batch(output_dir: str, all_metrics: MetricCollection,
                                 controller_type: str, batch: str) -> None:
    """ Saves the mean and standard deviation of each metric to a CSV file. """
    # Prepare the CSV file path
    filepath = os.path.join(output_dir, 'statistics.csv')
    
    # Open the CSV file for writing
    with open(filepath, mode='w', newline='') as file:
        writer = csv.writer(file) # csv.DictWriter(csvfile, fieldnames=fieldnames)
        file.write(f"Controller Type: {controller_type}, Batch: {batch} \n")
        
        writer.writerow(['Metric type', 'Mean', 'Std Dev'])
        
        # Write the mean and std for each metric
        for key, values in all_metrics.metric_collection.items():
            metrics_array = np.array(values)
            mean_value = np.mean(metrics_array, axis=0)
            std_value = np.std(metrics_array, axis=0)
            
            # Write the row for each metric
            writer.writerow([key, mean_value, std_value])


def validate_file(filepath: str) -> None:
    """ Validates if the input file exists and is not empty. """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Input file '{filepath}' does not exist.")
    if os.path.getsize(filepath) == 0:
        raise ValueError(f"Input file '{filepath}' is empty.")
    

