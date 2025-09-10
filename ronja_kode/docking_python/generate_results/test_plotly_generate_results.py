from generate_results.plotly_plotting import plot_traj_with_vessel_outline, plot_statistics_boxplots
from generate_results.plotly_plotting import plot_pose_components, plot_pos_heading_errors, plot_pose_component_errors
from generate_results.plotly_plotting import plot_nu, plot_speed_with_error
from generate_results.plotly_plotting import plot_accel_components, plot_lin_angular_accel
from generate_results.plotly_plotting import plot_lin_angular_control, plot_control_components
from generate_results.plotly_plotting import plot_jerk_components, plot_lin_angular_jerk

from generate_results.compute_metrics import compute_all_metric_types
from generate_results.read_write_files import read_and_process_logged_data, save_metrics_single_run
from generate_results.read_write_files import save_statistics_single_batch, read_and_process_statistics
from generate_results.read_write_files import validate_file, save_fig_plotly

from docking_algorithms.utils.data_logger import load_config
from generate_results.types import MetricCollection, Metric, StatisticsCollection
from generate_results.utils import expand_metric_collection

import os


BASE_DIR = '../masteroppgave-rapport/results/'
LOG_FILENAME = 'logged_data.csv'
METRICS_FILENAME = 'metrics.csv'
STATISTICS_FILENAME = 'statistics.csv'
PLOTTING_COLORS = load_config(cfg_name="plotting", cfg_path="../../generate_results")["plotly"]["plotting_colors"]
QUAYS_VESSEL_CFG = load_config(cfg_name="quays_vessel", cfg_path="../config")
ALL_CONTROLLERS = ["pid", "mpc"]
ALL_BATCHES = ["no_disturbances", "small_disturbances", "moderate_disturbances", "large_disturbances"]


# TODO: ensure that these are correct before plotting or computing metrics/statistics !!
CONTROLLER_TYPE = "mpc_pid" # pid or mpc
BATCH = "no_disturbances" # no_disturbances, small_disturbances, moderate_disturbances, large_disturbances
RUN_ID = 1 # 1 to n (n is to be decided)
QUAY_TYPE = "l_quay" # l_quay or h_quay. TODO: Maybe this will be part of the results hierarchy
RUN_INFO = 'ext_pt_2m_decreaseHeading_increaseCross_centerConstraintSide'


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
    metrics = compute_all_metric_types(data=processed_data)
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
        fig.show()
    else:
        for metric_type, fig in figures.items():
            fig_filename = f"boxplot_{metric_type}"
            save_fig_plotly(output_dir=BASE_DIR, fig_name=fig_filename, file_format="pdf", fig=fig)
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
def test_plot_traj_with_vessel_outline(show_fig: bool=True) -> None:
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "traj_with_vessel_outline"
    processed_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_traj_with_vessel_outline(data=processed_data, quay_type=QUAY_TYPE,
                                 colors=PLOTTING_COLORS, run_name = RUN_INFO,
                                 quays_vessel_cfg=QUAYS_VESSEL_CFG)
    if show_fig:
        fig.show()
    else:
        save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return


def test_plot_pose_component_errors(show_fig: bool=True) -> None:
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "pose_component_errors"
    processed_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_pose_component_errors(data=processed_data, colors=PLOTTING_COLORS)
    if show_fig:
        fig.show()
    # save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return


def test_plot_pose_components(show_fig: bool=True) -> None:
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "pose_components_plotly"
    processed_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_pose_components(data=processed_data, colors=PLOTTING_COLORS)
    if show_fig:
        fig.show()
    else:
        save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return


def test_plot_pos_heading_errors(show_fig: bool=True) -> None:
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "pos_heading_errors"
    processed_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_pos_heading_errors(data=processed_data, colors=PLOTTING_COLORS)
    if show_fig:
        fig.show()
    else:
        save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return
    

def test_plot_nu(show_fig: bool=True) -> None:
    """
    PASSED TEST
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "nu"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_nu(data=logged_data, colors=PLOTTING_COLORS)
    if show_fig:
        fig.show()
    else:
        save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return


def test_plot_speed_with_error(show_fig: bool=True) -> None:
    """
    PASSED TEST
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "speed_with_error"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_speed_with_error(data=logged_data, colors=PLOTTING_COLORS)
    if show_fig:
        fig.show()
    else:
        save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return


def test_plot_accel_components(show_fig: bool=True) -> None:
    """
    BODY acceleration
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    
    figure_filename = "accel_components"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_accel_components(data=logged_data, colors=PLOTTING_COLORS)
    if show_fig:
        fig.show()
    else:
        save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return

def test_plot_lin_angular_accel(show_fig: bool=True) -> None:
    """
    BODY acceleration
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    figure_filename = "lin_ang_accel"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_lin_angular_accel(data=logged_data, colors=PLOTTING_COLORS)
    if show_fig:
        fig.show()
    else:
        save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return


def test_plot_control_components(show_fig: bool=True) -> None:
    """
    BODY control forces and moments
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    figure_filename = "control_components"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_control_components(data=logged_data, colors=PLOTTING_COLORS)
    if show_fig:
        fig.show()
    else:
        save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return


def test_plot_lin_ang_control(show_fig: bool=True) -> None:
    """
    BODY control forces and moments
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    figure_filename = "lin_ang_control"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_lin_angular_control(data=logged_data, colors=PLOTTING_COLORS)
    if show_fig:
        fig.show()
    else:
        save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return


def test_plot_jerk_components(show_fig: bool=True) -> None:
    """
    BODY jerk
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    figure_filename = "jerk_components"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_jerk_components(data=logged_data, colors=PLOTTING_COLORS)
    if show_fig:
        fig.show()
    else:
        save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return

def test_plot_lin_angular_jerk(show_fig: bool=True) -> None:
    """
    BODY jerk
    """
    logged_data_filepath = os.path.join(RUN_DIR, LOG_FILENAME)
    figure_filename = "lin_ang_jerk"
    logged_data = read_and_process_logged_data(filepath=logged_data_filepath)
    fig = plot_lin_angular_jerk(data=logged_data, colors=PLOTTING_COLORS)
    if show_fig:
        fig.show()
    else:
        save_fig_plotly(output_dir=RUN_DIR, fig_name=figure_filename, file_format="pdf", fig=fig)
    return
    


def test_run_all_plots_and_metrics(show_fig: bool=True) -> None:
    test_plot_traj_with_vessel_outline(show_fig=show_fig)
    test_plot_pose_components(show_fig=show_fig)
    test_plot_pos_heading_errors(show_fig=show_fig)

    test_plot_nu(show_fig=show_fig)
    test_plot_speed_with_error(show_fig=show_fig)

    test_plot_accel_components(show_fig=show_fig)
    test_plot_lin_angular_accel(show_fig=show_fig)

    test_plot_control_components(show_fig=show_fig)
    test_plot_lin_ang_control(show_fig=show_fig)

    test_plot_jerk_components(show_fig=show_fig)
    test_plot_lin_angular_jerk(show_fig=show_fig)

    test_compute_metrics_for_single_run()

    return
