from generate_results.plotting import plot_vessel_trajectory, plot_statistics_boxplots
from generate_results.plotting import plot_pose_components, plot_pos_heading_errors, plot_pose_component_errors
from generate_results.plotting import plot_nu, plot_speed_with_error, plot_eta_dot
from generate_results.plotting import plot_accel_components, plot_lin_angular_accel
from generate_results.plotting import plot_lin_angular_control, plot_control_components
from generate_results.plotting import plot_jerk_components, plot_lin_angular_jerk, plot_traj_jerk_heatmat
from generate_results.plotting import plot_wind_forces_body, plot_heading_yawRate_speed, plot_accels_and_controls

from generate_results.compute_metrics import compute_all_metric_types
from generate_results.read_write_files import read_and_process_logged_data, save_metrics_single_run, read_and_process_wind_data
from generate_results.read_write_files import save_statistics_single_batch, read_and_process_statistics
from generate_results.read_write_files import validate_file, save_fig

from docking_algorithms.utils.data_logger import load_config
from generate_results.types import MetricCollection, Metric, StatisticsCollection
from generate_results.utils import expand_metric_collection

import os
import matplotlib.pyplot as plt
import pandas as pd


BASE_DIR = '../masteroppgave-rapport/results/'  # masteroppgave-rapport is the locally set up latex report
LOG_FILENAME = 'logged_data.csv'
METRICS_FILENAME = 'metrics.csv'
STATISTICS_FILENAME = 'statistics.csv'
PLOTTING_COLORS = load_config(cfg_name="plotting", cfg_path="../../generate_results")["plotting_colors"]
QUAYS_VESSEL_CFG = load_config(cfg_name="quays_vessel", cfg_path="../config")
ALL_CONTROLLERS = ["pid", "mpc_pid"]
ALL_BATCHES = ["no_disturbances", "with_disturbances"]


# TODO: ensure that these are correct before plotting or computing metrics/statistics !!
CONTROLLER_TYPE = "mpc_pid" # pid or mpc_pid
QUAY_TYPE = "l_quay" # l_quay or h_quay. TODO: Maybe this will be part of the results hierarchy
BATCH = "no_disturbances" # no_disturbances, with_disturbances
RUN_ID = 1 # 1 to n (n is to be decided)
RUN_INFO = 'baseline_mpc_left'


CONTROLLER_DIR = os.path.join(BASE_DIR, CONTROLLER_TYPE)
BATCH_DIR = os.path.join(CONTROLLER_DIR, BATCH)
RUN_DIR = os.path.join(BATCH_DIR, f"run{str(RUN_ID)}", RUN_INFO)


# ---------- METRICS BASED ON A SINGLE RUN ---------- #
def test_compute_metrics_for_single_run() -> None:
    """
    PASSED TEST

    Purpose: 
        - Compute metrics for the single run specified by CONTROLLER_TYPE, BATCH and RUN_ID 
        defined at the top of this file
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    validate_file(filepath=logged_data_filepath)
    
    processed_data = read_and_process_logged_data(filepath=logged_data_filepath)
    metrics = compute_all_metric_types(data=processed_data, controller_type=CONTROLLER_TYPE)
    save_metrics_single_run(output_dir=RUN_DIR, metrics_list=metrics,
                 controller_type=CONTROLLER_TYPE, batch=BATCH, run_id=RUN_ID,
                 quay_type=QUAY_TYPE)


def test_compute_metrics_and_statistics_for_batch() -> None:
    """
    PASSED TEST

    Purpose: 
        - Computes the metrics for each run of the BATCH specified at the top of this file
            - Stores metrics to files within each run folder
        - Computes the statistics (mean, standard deviation) for the BATCH
            - Stored statistics to file within the batch folder
    """
    NUM_RUNS = 3

    metric_collection = MetricCollection(metric_collection={})
    for i in range(NUM_RUNS):
        run_id = i+1
        metric_list : list[Metric] = [] # list of all computed metrics for ONE run
        target_dir = os.path.join(BATCH_DIR, f"run{i+1}")
        log_filepath = os.path.join(target_dir, LOG_FILENAME)
        validate_file(filepath=log_filepath)
        
        processed_data = read_and_process_logged_data(filepath=log_filepath)
        metric_list = compute_all_metric_types(data=processed_data)
        save_metrics_single_run(output_dir=target_dir, metrics_list=metric_list,
                    controller_type=CONTROLLER_TYPE, batch=BATCH, run_id=run_id,
                    quay_type=QUAY_TYPE)    # save metrics for the run to file

        # Include all metrics from a run in the MetricCollection
        metric_collection = expand_metric_collection(metric_list=metric_list, metric_collection=metric_collection)

    # Store metric type, mean and standarddeviation for the batch to file
    save_statistics_single_batch(output_dir=BATCH_DIR, all_metrics=metric_collection,
                                 controller_type=CONTROLLER_TYPE, batch=BATCH)
    


# ---------- THESE TWO COMPUTES AND PLOTS EVERYTHING ---------- #
def test_compute_all_metrics_and_statistics_for_one_controller() -> None:
    """
    PASSED TEST

    THIS IS THE MAIN TEST TO RUN
        - RUN THIS AFTER EVERY NEW SIMULATION TO ENSURE UPDATED METRICS AND STATISTICS

    Purpose: 
        - Computes the metrics for each run within ALL_BATCHES
            - Stores metrics to files within each run folder
        - Computes the statistics (mean, standard deviation) for the current BATCH
            - Stored statistics to file within the batch folder
    """
    for batch in ALL_BATCHES:
        # updating the global variables
        global BATCH_DIR, BATCH
        BATCH = batch
        BATCH_DIR = os.path.join(CONTROLLER_DIR, batch)

        test_compute_metrics_and_statistics_for_batch()


def test_plot_both_controller_statistics(show_fig=True) -> None:
    """
    PASSED TEST: everything except for the plotting - decide how to group stuff

    Purpose:
        - Produces box plot(s) (or similar - TBD)
            - Computes all metrics and statistics for all runs, batches and controllers before plotting
    """
    statistics_collection = StatisticsCollection(statistic_collection={})
    for c in ALL_CONTROLLERS:
        test_compute_all_metrics_and_statistics_for_one_controller()
        for b in ALL_BATCHES:
            stats_filepath = os.path.join(BASE_DIR, c, b, STATISTICS_FILENAME)
            statistics_collection = read_and_process_statistics(filepath=stats_filepath,
                                                                statistics_collection=statistics_collection)
            
    # fig is a collection of several figs - store them in a loop or something
    figures = plot_statistics_boxplots(statistics_collection=statistics_collection) # TODO: fix the plotting
    if show_fig:
        plt.show()
    else:
        for metric_type, fig in figures.items():
            fig_filename = f"boxplot_{metric_type}"
            save_fig(output_dir=BASE_DIR, fig_name=fig_filename, file_format="pdf")
    return
# --------------------------------------------------------------------------------------------------------- #

def test_compute_metrics_for_selected_runs() -> None:
    """
    PASSED TEST

    Purpose: 
        - Compute metrics for the runs specified by RUN_IDS, for the CONTROLLER_TYPE and BATCH
        defined at the top of this file
            - Stores the metrics to files within each run folder
    """
    RUN_IDS = [1, 2, 3] # list the id of the run to process
    for i, id in enumerate(RUN_IDS):
        target_dir = os.path.join(BATCH_DIR, f"run{id}")
        input_filepath = os.path.join(target_dir, LOG_FILENAME)
        validate_file(filepath=input_filepath)

        processed_data = read_and_process_logged_data(filepath=input_filepath)
        metrics = compute_all_metric_types(data=processed_data)
        save_metrics_single_run(output_dir=target_dir, metrics_list=metrics,
                    controller_type=CONTROLLER_TYPE, batch=BATCH, run_id=id,
                    quay_type=QUAY_TYPE)
        metrics.clear()


def test_compute_metrics_and_statistics_for_all_controllers():
    """
    PASSED TEST

    Purpose:
        - Compute metrics and statistics for all runs in all batches for all controllers
    """
    for c in ALL_CONTROLLERS:
        global CONTROLLER_TYPE, CONTROLLER_DIR
        CONTROLLER_TYPE = c
        CONTROLLER_DIR = os.path.join(BASE_DIR, c)
        test_compute_all_metrics_and_statistics_for_one_controller()


# ---------- PLOTS BASED ON A SINGLE RUN ---------- #
def test_plot_vessel_trajectory(show_fig: bool=False, plot_contour=30) -> None:
    """
    PASSED TEST
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "traj"
    processed_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_vessel_trajectory(data=processed_data, quay_type=QUAY_TYPE,
                                 colors=PLOTTING_COLORS, quays_vessel_cfg=QUAYS_VESSEL_CFG,
                                 controller_type=CONTROLLER_TYPE, plot_contour=plot_contour)
    fig.tight_layout()
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return

def test_plot_traj_jerk_heatmat(show_fig: bool=False) -> None:
    """

    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "traj_jerk_heatmap"
    processed_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_traj_jerk_heatmat(data=processed_data, quay_type=QUAY_TYPE, colors=PLOTTING_COLORS,
                                 quays_vessel_cfg=QUAYS_VESSEL_CFG)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return


def test_plot_pose_component_errors(show_fig: bool=False) -> None:
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "pose_component_errors"
    processed_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_pose_component_errors(data=processed_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return


def test_plot_pose_components(show_fig: bool=False) -> None:
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "pose_components"
    processed_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_pose_components(data=processed_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return


def test_plot_pos_heading_errors(show_fig: bool=False) -> None:
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "pos_heading_errors"
    processed_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_pos_heading_errors(data=processed_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return
    

def test_plot_nu(show_fig: bool=False) -> None:
    """
    PASSED TEST
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "nu"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_nu(data=logged_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return


def test_plot_eta_dot(show_fig: bool=False) -> None:
    """
    PASSED TEST
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "eta_dot"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_eta_dot(data=logged_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return


def test_plot_speed_with_error(show_fig: bool=False) -> None:
    """
    PASSED TEST
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "speed_with_error"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_speed_with_error(data=logged_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return


def test_plot_accel_components(show_fig: bool=False) -> None:
    """
    BODY acceleration
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "accel_components"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_accel_components(data=logged_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    fig.tight_layout()
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return

def test_plot_lin_angular_accel(show_fig: bool=False) -> None:
    """
    BODY acceleration
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    figure_filename = "lin_ang_accel"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_lin_angular_accel(data=logged_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return


def test_plot_control_components(show_fig: bool=False) -> None:
    """
    BODY control forces and moments
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    figure_filename = "control_components"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_control_components(data=logged_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    fig.tight_layout()
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return


def test_plot_lin_ang_control(show_fig: bool=False) -> None:
    """
    BODY control forces and moments
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    figure_filename = "lin_ang_control"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_lin_angular_control(data=logged_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return


def test_plot_jerk_components(show_fig: bool=False) -> None:
    """
    BODY jerk
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    figure_filename = "jerk_components"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_jerk_components(data=logged_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return

def test_plot_lin_angular_jerk(show_fig: bool=False) -> None:
    """
    BODY jerk
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    figure_filename = "lin_ang_jerk"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_lin_angular_jerk(data=logged_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return
    

def test_plot_wind_forces(show_fig: bool=False) -> None:
    """ 
    BODY wind forces
    """
    logged_data_filepath = os.path.join(RUN_DIR, "wind_forces.csv")
    figure_filename = "wind_forces_body"
    logged_data = read_and_process_wind_data(filepath=logged_data_filepath)
    fig = plot_wind_forces_body(data=logged_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return

def test_plot_heading_yawRate_speed(show_fig: bool=False) -> None:
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    figure_filename = "head_yawRate_speed"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_heading_yawRate_speed(data=logged_data, colors=PLOTTING_COLORS, controller_type=CONTROLLER_TYPE)
    fig.tight_layout()
    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf")
    return

def test_plot_all_windForces_accels_control(show_fig: bool=False) -> None:
    """
    Plot one figure with wind forces from all LOS or MPC runs with disturbances,
    and the same for accelerations and control
    """
    LEGEND_FONTSIZE = 12

    wind_cases_mpc = {
        "speed 7": {
            "folder": "fullMPC_speed7_dir84_left",
            "labels": [r"$X_{\text{wind}, 7 m/s}$", r"$Y_{\text{wind}, 7 m/s}$", r"$N_{\text{wind}, 7 m/s}$"],
            "color": (0.0, 0.3647, 0.9686)
        },
        "speed 11": {
            "folder": "fullMPC_speed11_dir84_left",
            "labels": [r"$X_{\text{wind}, 11 m/s}$", r"$Y_{\text{wind}, 11 m/s}$", r"$N_{\text{wind}, 11 m/s}$"],
            "color": (0.9216, 0.0863, 0.0)
        }
    }

    wind_cases_los = {
        "speed 7": {
            "folder": "speed7_dir84_left",
            "labels": [r"$X_{\text{wind}, 7 m/s}$", r"$Y_{\text{wind}, 7 m/s}$", r"$N_{\text{wind}, 7 m/s}$"],
            "color": (0.0, 0.3647, 0.9686)
        },
        "speed 11": {
            "folder": "speed11_dir84_left",
            "labels": [r"$X_{\text{wind}, 11 m/s}$", r"$Y_{\text{wind}, 11 m/s}$", r"$N_{\text{wind}, 11 m/s}$"],
            "color": (0.9216, 0.0863, 0.0)
        }
    }
    if CONTROLLER_TYPE == "mpc_pid":
        wind_cases = wind_cases_mpc
    else: # pid (los)
        wind_cases = wind_cases_los
    
    wind_fig_filename = "wind_forces_body"
    output_dir = os.path.join(BASE_DIR, CONTROLLER_TYPE, "with_disturbances", "run1")

    # --- Wind Forces (only selected cases) ---
    fig_wind, axes_wind = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    for wind_speed_case, info in wind_cases.items():
        path = os.path.join(output_dir, info["folder"], "wind_forces.csv")
        df = pd.read_csv(path)

        time = df.iloc[:, 0]
        X_wind = df.iloc[:, 1]
        Y_wind = df.iloc[:, 2]
        N_wind = df.iloc[:, 3]

        axes_wind[0].plot(time, X_wind, label=info["labels"][0], color=info["color"], linewidth=1.5)
        axes_wind[1].plot(time, Y_wind, label=info["labels"][1], color=info["color"], linewidth=1.5)
        axes_wind[2].plot(time, N_wind, label=info["labels"][2], color=info["color"], linewidth=1.5)

    axes_wind[0].set_ylabel(r'Force [$N$]')
    axes_wind[0].legend(loc='best', fontsize=LEGEND_FONTSIZE)
    axes_wind[0].grid(True)

    axes_wind[1].set_ylabel(r'Force [$N$]')
    axes_wind[1].legend(loc='best', fontsize=LEGEND_FONTSIZE)
    axes_wind[1].grid(True)

    axes_wind[2].set_ylabel(r'Moment [$Nm$]')
    axes_wind[2].set_xlabel(r'Time [$s$]')
    axes_wind[2].legend(loc='best', fontsize=LEGEND_FONTSIZE)
    axes_wind[2].grid(True)
    plt.tight_layout()

    if show_fig:
        plt.show()
    else:
        save_fig(output_dir=output_dir, fig_name=wind_fig_filename, file_format="pdf")

    plot_accels_and_controls(output_dir=output_dir, show_fig=show_fig, suffix="_left", controller_type=CONTROLLER_TYPE)
    plot_accels_and_controls(output_dir=output_dir, show_fig=show_fig, suffix="_right", controller_type=CONTROLLER_TYPE)
    return


def test_run_all_plots_and_metrics(show_fig: bool=False) -> None:
    test_plot_vessel_trajectory(show_fig=show_fig, plot_contour=30)

    # test_plot_nu(show_fig=show_fig)
    # test_plot_eta_dot(show_fig=show_fig)

    test_plot_accel_components(show_fig=show_fig)
    # test_plot_lin_angular_accel(show_fig=show_fig)

    test_plot_control_components(show_fig=show_fig)
    # test_plot_lin_ang_control(show_fig=show_fig)

    test_plot_heading_yawRate_speed(show_fig=show_fig)

    # test_compute_metrics_for_single_run()

    if BATCH == "with_disturbances":
        test_plot_wind_forces(show_fig=show_fig)

    # test_plot_pose_components(show_fig=show_fig)
    # test_plot_pose_component_errors(show_fig=show_fig)
    # test_plot_pos_heading_errors(show_fig=show_fig)
    # test_plot_speed_with_error(show_fig=show_fig)

    # test_plot_jerk_components(show_fig=show_fig)
    # test_plot_lin_angular_jerk(show_fig=show_fig)
    return


def test_replot_all_runs(show_fig: bool=False) -> None:
    mpc_targets_noDisturbances = [
    "baseline_mpc_left",
    "frontCenter_left",
    "frontCenter_sideCenter_left",
    "frontCenter_sideCenter_right",
    # "frontCenter_sideCorner_left",
    "frontCenter_sideCorner_right",
    # "frontCorner_left",
    # "guideline_frontCenter_left",
    # "guideline_frontCenter_right",
    "guideline_frontCenter_sideCenter_left",
    "guideline_frontCenter_sideCenter_right",
    # "guideline_left",
    # "guideline_right",
    # "sideCenter_left",
    # "sideCenter_right",
    # "sideCorner_left",
    # "sideCorner_right",
    ]

    mpc_targets_withDisturbances = [
        "fullMPC_speed7_dir84_left",
        "fullMPC_speed7_dir84_right",
        "fullMPC_speed7_dir264_left",
        "fullMPC_speed7_dir264_right",
        "fullMPC_speed11_dir84_left",
        "fullMPC_speed11_dir84_right",
        "fullMPC_speed11_dir264_left",
        "fullMPC_speed11_dir264_right",
    ]

    los_targets_noDisturbances = ["los_left", "los_right"]
    los_targets_withDisturbances = ["speed7_dir84_right", "speed7_dir84_left",
                                    "speed7_dir264_right", "speed7_dir264_left",
                                    "speed11_dir84_right", "speed11_dir84_left",
                                    "speed11_dir264_right", "speed11_dir264_left"]

    original_globals = globals().copy()
    for controller in ALL_CONTROLLERS:
        controller_type = controller
        globals()['CONTROLLER_TYPE'] = controller_type
        globals()['CONTROLLER_DIR'] = os.path.join(BASE_DIR, CONTROLLER_TYPE)

        for batch in ALL_BATCHES:
            globals()['BATCH'] = batch
            globals()['BATCH_DIR'] = os.path.join(CONTROLLER_DIR, BATCH)

            if batch == "no_disturbances" and controller_type == "pid":
                run_descriptions = los_targets_noDisturbances
            elif batch == "with_disturbances" and controller_type == "pid":
                run_descriptions = los_targets_withDisturbances
            elif batch == "no_disturbances" and controller_type == "mpc_pid":
                run_descriptions = mpc_targets_noDisturbances
            else:  # with_disturbances and mpc_pid
                run_descriptions = mpc_targets_withDisturbances

            for run_info in run_descriptions:
                globals()['RUN_INFO'] = run_info
                globals()['RUN_DIR'] = os.path.join(BATCH_DIR, f"run{str(RUN_ID)}", run_info)

                if not os.path.isdir(RUN_DIR):
                        print(f"Skipping missing folder: {RUN_DIR}")
                        continue
                
                test_run_all_plots_and_metrics(show_fig=show_fig)
    globals().update(original_globals)
    return
